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

const int NMOT = 8;

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
  volatile float roll, pitch, yaw;//16
  //r pitchdot, rolldot, yawdot
  volatile float rolldot, pitchdot, yawdot;//28
  // Motor positions
  volatile float q[8];//60
  // Motor currents
  // volatile float magx, magy, magz; // 72 // for debug only 
  volatile float torque[8];//92
  // forward velocity
  volatile float xd;//96
  // forward velocity
  volatile float Vbatt;//100
  // 8-bit discrete mode
  volatile uint8_t mode;//101
} __attribute__ ((packed));
extern volatile LogVector X;

void halInit();
void halUpdate();
void enable(bool);
void errorStop(const char *msg);
extern bool halHeartbeatEnabled;

#endif

