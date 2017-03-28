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
const bool REMOTE_RC_6CH = true;//false if only 4 channels connected
const bool REMOTE_RC_EXTRA_CHANNELS = true; // only for remoteRC: true if 6 channels connected
Peripheral *imu = &imuMPU6000;// imuVN100 / imuMPU6000
// ====== To save compile time if not using MPU6000, comment next two lines =====
#include <MPU6000.h>
#include <Eigen.h>

// This must be set per robot
// const float motZeros[8] = {3.408, 6.238, 4.169, 0.227, 3.744, 3.147, 3.457, 3.596};//G Mini
// const float motZeros[8] = {0.631, 4.076, 1.852, -2.897, 8.115, 1.169, 1.07, 6.252};//Aaron Mini
const float motZeros[8] = {0.631, 4.076, 1.852, 3.414, 1.817, 1.169, 1.078, 6.252};//Aaron Mini



// Behavior array: add behaviors here. First one in the array is the starting behavior.
// Make sure the #include is in Remote.h
const int NUM_BEHAVIORS = 4;
Behavior *behaviorArray[NUM_BEHAVIORS] = {&bound, &dig, &force, &traj};

// ======================================================================

volatile uint32_t controlTime = 0;

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
  behavior->update();

  // Remote: set parameters, stop behavior
  remote->updateInterrupt();

  controlTime = micros() - tic;
}

void setup() {
  halInit();

  // Should use 500Hz when 4 motors/port, try 1000 with 2/port
  attachTimerInterrupt(0, controlLoop, CONTROL_RATE);
  attachTimerInterrupt(1, debug, 20);

  if (remote != &remoteComputer) {
    enable(true);
    // for testing
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
  
  delay(10000);
}

void loop() {
  // Behavior starting/signalling code, and nunchuck update go here
  remote->updateLoop();
}
