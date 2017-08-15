/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#include <Arduino.h>
#include <WMath.h>
#include "HAL.h"
#include "Remote.h"
#include "IMUObject.h"
volatile float log_flag = 0;
const uint8_t VsourcePin = PF2;
DLPF VsourceF;
// LEDs
const uint8_t led0 = PD1, led1 = PD0;
// heartbeat
volatile uint32_t nIters = 0;
// volatile uint32_t lastPrint = 0;
int hbFreq = CONTROL_RATE/10;
int hbOnFreq = CONTROL_RATE/20;
bool halHeartbeatEnabled = true;

#if USE_BUS
// mblc0.4
DxlNode master9(PA12, Serial4);
DxlNode master8(PA15, Serial5);

DxlMotor M[NMOT];
const uint8_t NMOT9 = 4, NMOT8 = 4;
const uint8_t ids9[] = {0, 1, 2, 3};
const uint8_t ids8[] = {4, 5, 6, 7};

// bus vars
volatile uint32_t totalPings=0, totalFailures = 0;
volatile uint32_t ping = 0;//, lastPrint = 0;
const int CONTROL_RATE = 400;

#else//PWM
const uint8_t pwmPin[] = {PE9, PE11, PE13, PE14, PA0, PD4, PD7, PD6, PB4, PB5};
const uint8_t posPin[] = {PD12, PD13, PD14, PD15, PC6, PC7, PC8, PC9, PE2, PE3};
// const uint8_t curPin[] = {PD8, PB2, PA13, PB1, PB0, PD5, PF9, PF10, PE4, PE5};
const uint8_t motorPort[9] = {0, 1, 2, 3, 4, 5, 6, 7, 8};//MINI
BlCon34 M[NMOT];
const int CONTROL_RATE = 1000;
#endif

// OpenLog
BulkSerial openLog(MBLC_OPENLOG);
volatile uint32_t lastOLwrite = 0;

// STATE
volatile LogVector X;

float Kt = 0.0954;
float res = 0.186;
float Vsource = 16;
float curLim = 60;

// Legs
// MINITAUR
const int8_t dir[] = {1, 1, 1, 1, -1, -1, -1, -1};
MinitaurLeg leg[4] = {MinitaurLeg(&M[1], &M[0]),//0
MinitaurLeg(&M[3], &M[2]), //1
MinitaurLeg(&M[4], &M[5]), //2
MinitaurLeg(&M[6], &M[7])};//3
// MEGATAUR
// const int8_t dir[] = {-1, -1, 1, 1, 1, 1, -1, -1};
// MinitaurLeg leg[4] = {MinitaurLeg(&M[1], &M[0]),//0
// MinitaurLeg(&M[2], &M[3]), //1
// MinitaurLeg(&M[4], &M[5]), //2
// MinitaurLeg(&M[7], &M[6])};//3
float ux[4], uz[4];

// Safety check
// volatile uint32_t legStuckTimer[4];
volatile uint32_t legStuckTimer[8];

void enable(bool flag) {
  for (int i=0; i<4; ++i)
    leg[i].enable(flag);
}

void errorStop(const char *msg) {
  enable(false);
  digitalWrite(led0, HIGH);
  while (1) {
    digitalWrite(led1, LOW);
    delay(50);
    digitalWrite(led1, HIGH);
    Serial1 << msg << endl;
    delay(1000);
  }
}

void halInit() {
  // leds
  
  pinMode(led0, OUTPUT);
  pinMode(led1, OUTPUT);
  digitalWrite(led1, HIGH);
  // Vsource
  VsourceF.init(0.999, CONTROL_RATE, DLPF_SMOOTH);
  pinMode(VsourcePin, INPUT_ANALOG);

  Motor::updateRate = CONTROL_RATE;
  Motor::velSmooth = 0.55;
  MinitaurLeg::useLengths = false;
#if USE_BUS
  Serial4.irqnPriority = 2;
  Serial5.irqnPriority = 1;
  // Serial1.irqnPriority = 10;
  // Serial3.irqnPriority = 9;

  master8.init(NULL);
  master9.init(NULL);

  // init motors
  for (int i=0; i<NMOT9; ++i) {
    uint8_t id = ids9[i];
    M[id].init(&master9, id, motZeros[id], (id<4) ? 1 : -1);
  }
  for (int i=0; i<NMOT8; ++i) {
    uint8_t id = ids8[i];
    M[id].init(&master8, id, motZeros[id], (id<4) ? 1 : -1);
  }
  delay(1000);
  // Establish comms
  // // Need to call update on motors at least once and then resetOffset()
  // int i=0;
  // while (i < NMOT) {
  //   M[i].update();
  //   if (M[i].updated())
  //     i++;
  //   else {
  //     Serial1 << "ERROR: bus communication failed with " << i << "\n";
  //   }
  //   digitalWrite(led0, TOGGLE);
  //   digitalWrite(led1, TOGGLE);
  //   delay(50);
  // }
  // for (int i=0; i<NMOT; ++i)
  //   M[i].resetOffset();
  // Legs: update motors separately
  for (int i=0; i<4; ++i)
    leg[i].autoUpdate = false;
#else

  // wait a bit (??)
  delay(750);

  for (int i=0; i<NMOT; ++i) {
    uint8_t port = motorPort[i];
    M[i].init(pwmPin[port], posPin[port], motZeros[i], dir[i]);
    M[i].setTorqueEstParams(Kt, res, Vsource, curLim);

    // current reading: these are on EXTI, and on T15, T3 (for J8 and J9)
    // try to set to lower priority
    // TIMER_IC_PRIORITY = 1;
    // pinMode(curPin[port], (port<6) ? PWM_IN_EXTI : PWM_IN);
    // TIMER_IC_PRIORITY = 0;
  }
#endif//if USE_BUS

  Serial1.begin(115200);

  // REMOTE
  remote->begin();
  imu->begin();

  // Try to init openlog (don't stop if no SD card)
  openLog.begin(115200, sizeof(X), (void *)&X, 0);
//  openLog.initOpenLog("t,r,p,y,rd,pd,yd,q0,q1,q2,q3,q4,q5,q6,q7,u0,u1,u2,u3,u4,u5,u6,u7,xd,Vb,mo", "IfffffffffffffffffffffffffB"); // original

  //openLog.initOpenLog("t,r,p,y,rd,pd,yd,q0,q1,q2,q3,q4,q5,q6,q7,q8,u0,u1,u2,u3,u4,u5,u6,u7,u8,p0,p1,p2,p3,p4,p5,p6,p7,p8,xd,Vb,log,mo", "IfffffffffffffffffffffffffffffffffffBB");
  //openLog.initOpenLog("t,r,p,y,rd,pd,yd,q0,q1,q2,q3,q4,q5,q6,q7,q8,dq0,dq1,dq2,dq3,dq4,dq5,dq6,dq7,dq8,u0,u1,u2,u3,u4,u5,u6,u7,u8,p0,p1,p2,p3,p4,p5,p6,p7,p8,xd,Vb,mo", "IffffffffffffffffffffffffffffffffffffffffffffB");
  openLog.initOpenLog("t,p,p0,p1,p2,p3,p4,p5,p6,p7,p8,log1,mo,r,y,xd,rd,pd,yd", "IfffffffffffBffffff");                    
  //openLog.initOpenLog("t,r,p,y,rd,pd,yd,q0,q1,q2,q3,q4,q5,q6,q7,q8,dq0,dq1,dq2,dq3,dq4,dq5,dq6,dq7,dq8,u0,u1,u2,u3,u4,u5,u6,u7,u8,p0,p1,p2,p3,p4,p5,p6,p7,p8,Vb,log1,mo", "IffffffffffffffffffffffffffffffffffffffffffffB");

  // openLog.initOpenLog("t,r,p,y,rd,pd,yd,q0,q1,q2,q3,q4,q5,q6,q7,magx,magy,magz,u3,u4,u5,u6,u7,xd,Vb,mo", "IffffffffffffffffffffffffB");

  // Hardware setup done
  digitalWrite(led0, HIGH);
  digitalWrite(led1, HIGH);
}

void halUpdate() {
  // MOTORS
  for (int i=0; i<4; ++i)
    leg[i].update();
  // for PWM the leg update will update this
  // for bus, autoUpdate is false, so halUpdate() updates motors
  M[8].updateTail();
//  M[8].update();
#if USE_BUS
  // update one from each master at a time
  for (uint8_t i=0; i<4; ++i) {
    // send command
    if (i < NMOT8)
      M[ids8[i]].update();
    if (i < NMOT9)
      M[ids9[i]].update();
    // wait for response
    if (i < NMOT8)
      M[ids8[i]].update2();
    if (i < NMOT9)
      M[ids9[i]].update2();
  }
#endif//if USE_BUS

  // SAFETY CHECK
  for (int i=0; i<8; ++i) {
    // legs get stuck
    float val = M[i].getOpenLoop();
    if (fabsf(val) > 0.5) {
      legStuckTimer[i]++;
    } else {
      legStuckTimer[i] = 0;
    }
    // if stuck for 1 seconds, disable
    if (legStuckTimer[i] > CONTROL_RATE) {
      enable(false);
      hbFreq = CONTROL_RATE;
      hbOnFreq = CONTROL_RATE/10;
    }
  }

  // IMU
  imu->updateInterrupt();

  // Vsource
  // 3.3V, Volt div 470 & 10k, 12bit. So 3.3/4096*(10470/470) = 0.01794745262
  // empirical tuning: 
  //X.Vbatt = VsourceF.update(analogRead(VsourcePin)) * 0.02009387094;


  // totalPings+=NMOT;
  // ping = micros() - tic;

  // LOGGING
  X.t = millis();
  X.log1 = log_flag;
  for (int i=0; i<NMOT; ++i) {
    //X.q[i] = M[i].getPosition();
    //X.dq[i] = M[i].getVelocity();
//     float rawCur = 0;
// #if USE_BUS
//     rawCur = M[i].getCurrent();
// #else
//     // rawCur = map(pwmIn(curPin[motorPort[i]]), 0.1, 0.9, 0, 1);
// #endif
    // convert to multiples of 0.1A? current01*3.3/(0.5e-3*40)*10 = 1650
    // bias should be about 825
    // X.cur[i] = (uint16_t)(rawCur * 1650);

    //X.torque[i] = M[i].getTorque(); //INCLUDE IN NORMAL OPERATION
    //X.command[i] = M[i].getOpenLoop();
    // Estimate power at each of the legs
    
    X.power[i] = Vsource*M[i].getOpenLoop()*(Vsource*M[i].getOpenLoop() - M[i].getVelocity()*Kt)/res;
//    if (X.power[i] <= 0 ) X.power[i] = 0;  // Ignore negative power
  }
  for (int i=0; i<4; ++i) {
    // NOTE ux>0 when leg pushed "back", uz>0 when pushed "up"
    leg[i].getToeForceXZ(X.pitch, ux[i], uz[i]);
  }
  // mode is set elsewhere
  if (X.t - lastOLwrite > 9) {
    // openLog.write((const uint8_t *)&X);
    lastOLwrite = X.t;
    X.mode = ((IMUVN100 *)imu)->errId;
    openLog.write();
  }

  // heartbeat
  nIters++;
  digitalWrite(led0, (nIters % hbFreq < hbOnFreq) && halHeartbeatEnabled ? LOW : HIGH);
}


