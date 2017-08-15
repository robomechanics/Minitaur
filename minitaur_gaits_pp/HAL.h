/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#ifndef HAL_h
#define HAL_h

#include <stdint.h>

// TODO stop using preprocessor
#define USE_BUS 0

// globals defined in Remote.cpp, for use by behaviors
extern float speedDes, yawDes, latDes, vertDes;
extern uint8_t remoteKnob;
// defined in HAL.cpp
extern float ux[4], uz[4];
// defined in .ino
extern const float motZeros[];
extern volatile float log_flag;
const int NMOT = 9;

// MOTORS
#if USE_BUS
#include <DxlMotor.h>
extern DxlMotor M[NMOT];
#else
#include <Motor.h>
extern BlCon34 M[NMOT];
#endif
#include <BulkSerial.h>
#include <MinitaurLeg.h>

// OTHER PERIPHERALS
extern const uint8_t led0, led1;
extern const int CONTROL_RATE;
extern MinitaurLeg leg[4];
extern BulkSerial openLog;

struct LogVector {
  uint16_t align = 0xbbaa;//endianness reverses bytes to aabb
  // regular log
  volatile uint32_t t;//4
  
  
  // pitch = body pitch
  volatile float pitch;
  //volatile float roll, pitch, yaw;//16
  //r pitchdot, rolldot, yawdot
  
  // Motor positions
  //volatile float q[NMOT];//60
  // Motor velocities
  //volatile float dq[NMOT];//60
  // Motor currents
  // volatile float magx, magy, magz; // 72 // for debug only 
//  volatile float torque[NMOT];//92
  //volatile float command[NMOT];//92
  volatile float power[NMOT];//92
  // forward velocity
  //96
  volatile float log1;
 
   // 0 or 1 for if we are recording
  volatile uint8_t mode;//101
  
   volatile float roll, yaw;
  // forward velocity
  
  // 8-bit discrete mode
  volatile float xd;
  volatile float rolldot, pitchdot, yawdot;//28
  //volatile float Vbatt;//100
  
  
} __attribute__ ((packed));
extern volatile LogVector X;

void halInit();
void halUpdate();
void enable(bool);
void updateLogTag(int);
void errorStop(const char *msg);
extern bool halHeartbeatEnabled;

#endif

