/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#include <Motor.h>
// #if USE_BUS
// #include <DxlNode.h>
// #include <DxlMotor.h>
// #endif
#include <SPI.h>
#include <VN100.h>
#include <BulkSerial.h>
#include <Behavior.h>
#include "HAL.h"
#include "Remote.h"
#include "IMUObject.h"

// ROBOT CONFIGURATION OPTIONS ==========================================

// Change the upload port (in Arduino or Makefile)

Peripheral *remote = &remoteRC;// remoteRC / remoteComputer
const bool REMOTE_RC_EXTRA_CHANNELS = true; // only for remoteRC: true if 6 channels connected
Peripheral *imu = &imuVN100;// imuVN100 / imuMPU6000
// ====== To save compile time if not using MPU6000, comment next two lines =====
#include <MPU6000.h>
#include <Eigen.h>

// This must be set per robot
const float motZeros[8] = {0.379, 5.197, 3.540, 5.827, 5.669, 4.696, 1.183, 6.087};//Penn Mini

// Behavior array: add behaviors here. First one in the array is the starting behavior.
// Make sure the #include is in Remote.h
const int NUM_BEHAVIORS = 1;
Behavior *behaviorArray[NUM_BEHAVIORS] = {&bound};

// ======================================================================

volatile uint32_t controlTime = 0;
// For "soft start"
bool bPwrOnStateObtained = false;
uint32_t tPwrOnAnim0 = 0, tPwrOnAnimEnd = 3000;
float pwrOnExt[4], pwrOnAng[4];

void debug() {
  Serial1 << X.t << "\t";
  // Serial1 << controlTime << "\t";
  // Serial1 << pronk.getToeForceRadial();
  // Serial1 << yawDes;

  // IMU
  // Serial1 << X.roll << "\t" << X.pitch << "\t" << X.yaw << "\t";
  // Serial1 << rolldot << "\t" << pitchdot << "\t";
  // Serial1 << trot.d03.att << "\t" << trot.d21.att << "\t" << trot.d03.datt << "\t" << trot.d21.datt;

 //  // RC RECEIVER
 // for (int i=0; i<RemoteRC::NRECPINS; ++i)
 //   Serial1 << remoteRC.rcCmd[i] << "\t";
  // Serial1 << speedDes << "\t" << yawDes;

  // MOTORS
  for (int i=0; i<NMOT; ++i) {
   // Serial1 << M[i].id << "\t";
    // Serial1 << M[i].failures << "\t";
    Serial1 << _FLOAT(M[i].getRawPosition(),3) << "\t";
   // Serial1 << M[i].getTorque() << "\t";
    // Serial1 << X.cur[i] << "\t";
    // int period=0, pw=0;
    // pwmInRaw(M[i].inPin, &period, &pw);
    // Serial1 << period << "\t" << pw << "\t";
  }

  // LEG
  // for (int i=0; i<4; ++i) {
  //   // Serial1 << "[" << leg[i].getPosition(EXTENSION) << ","  << leg[i].getPosition(ANGLE) << "]\t";
  //   float ur, uth;
  //   leg[i].getToeForce(ur, uth);
  //   if (i==2 || i==3) ur = -ur;
  //   Serial1 << ur << "\t";
  // }

  Serial1 << "\n";
}

void controlLoop() {
  uint32_t tic = micros();
  halUpdate();
  // TEST individual motors
  // for (int i=0; i<NMOT; ++i) {
  //   M[i].enable(true);
  //   M[i].setGain(0.5);
  //   M[i].setPosition(1.5);
  //   // M[i].setOpenLoop(0.1);
  // }

  // BEHAVIOR
  // "soft start"
  if ((behavior == &bound) && millis() < tPwrOnAnimEnd) {
    if (!bPwrOnStateObtained) {
      for (int i=0; i<4; ++i) {
        pwrOnAng[i] = leg[i].getPosition(ANGLE);
        pwrOnExt[i] = leg[i].getPosition(EXTENSION);
      }
      bPwrOnStateObtained = true;
      tPwrOnAnim0 = millis();
    }
    for (int i=0; i<4; ++i) {
      bool bFront = (i==0) || (i==2);
      leg[i].setGain(EXTENSION, 0.2);
      leg[i].setGain(ANGLE, 0.4);
      if (millis() < 2000)
        leg[i].setGain(ANGLE, 0.05);
      // these two depend on the behavior: could make the behavior return these
      // for now these are reasonable for bound
      float behavExtDes = 1.5;//(behavior == &bound) ? 1.5 : 1.0;
      float behavAngDes = 0.0;
      leg[i].setPosition(EXTENSION, map(constrain(millis(),0,3000),tPwrOnAnim0,3000,pwrOnExt[i],behavExtDes));
      float ang0 = pwrOnAng[i];
      // avoid intersecting the body
      if (bFront && pwrOnAng[i] > HALF_PI)
        ang0 -= TWO_PI;
      else if (!bFront && pwrOnAng[i] < -HALF_PI)
        ang0 += TWO_PI;
      leg[i].setPosition(ANGLE, map(constrain(millis(),0,tPwrOnAnimEnd),tPwrOnAnim0,tPwrOnAnimEnd,ang0,behavAngDes));
    }
  }
  else {
    behavior->update();
  }

  // Remote: set parameters, stop behavior
  remote->updateInterrupt();

  controlTime = micros() - tic;
}

void setup() {
  halInit();

  // Should use 500Hz when 4 motors/port, try 1000 with 2/port
  attachTimerInterrupt(0, controlLoop, CONTROL_RATE);
  // attachTimerInterrupt(1, debug, 20);

  if (remote != &remoteComputer) {
    enable(true);
    // for testing
    bound.autopilot(false);
    // antiphase: +0.1 good for weights, smaller magnitude better for no weights, but still noisy
    // bound.setPhaseControlGain(0.05);
  }

  // // test no remote
  // delay(10000);
  // behavior->begin();
  // digitalWrite(led1, LOW);
  // delay(5000);
  // behavior->end();
  // digitalWrite(led1, HIGH);
}

void loop() {
  // Behavior starting/signalling code, and nunchuck update go here
  remote->updateLoop();
}
