/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#include <Motor.h>
#include <SPI.h>
#include <VN100.h>
#include <BulkSerial.h>
#include <Behavior.h>
#include "HAL.h"
#include "Remote.h"
#include "SoftStart.h"
#include "IMUObject.h"
// ====== To save compile time if not using MPU6000, comment next two lines =====
// #include <MPU6000.h>
// #include <Eigen.h>

// ROBOT CONFIGURATION OPTIONS ==========================================

// Change the upload port (in Arduino or Makefile)

Peripheral *remote = &remoteRC;// remoteRC / remoteComputer
const bool REMOTE_RC_6CH = true;//false if only 4 channels connected
Peripheral *imu = &imuVN100;// imuVN100 / imuMPU6000
// ====== To save compile time if not using MPU6000, comment next two lines =====
// #include <MPU6000.h>
// #include <Eigen.h>

// This must be set per robot zeros must be checked before running!
//const float motZeros[8] = {0.631, 4.076, 1.852, 3.414, 1.817, 1.169, 1.078, 6.252}; //RML Mini
const float motZeros[8] = {2.041, 1.616, 5.522, 2.484, 1.712, 5.356, 0.652, 2.017}; // MLab Mini

// Behavior array: add behaviors here. First one in the array is the starting behavior.
// Make sure the #include is in Remote.h
const int NUM_BEHAVIORS = 4;
Behavior *behaviorArray[NUM_BEHAVIORS] = {&bound, &walk, &dig, &pushwalk};

// ======================================================================

volatile uint32_t controlTime = 0;
SoftStart softStart;

void debug() {
  // TEST
  Serial1 << X.t << "\t";
  Serial1 << controlTime << "\t";// Make sure < 1000 (assuming CONTROL_RATE = 1000)!

  // Serial1 << remoteRC.throttle << "\t";

  // Serial1 << walk.frac << "\t";

  // Battery ------------------------------
  // Serial1 << X.Vbatt << "\t";// If resistor is not populated, will get > 50

  // IMU --------------------------------
  // Serial1 << X.roll << "\t" << X.pitch << "\t" << X.yaw << "\t";
  // Serial1 << X.rolldot << "\t" << X.pitchdot << "\t" << X.yawdot;

  // // RC RECEIVER --------------------
  // for (int i=0; i<RemoteRC::NRECPINS; ++i)
  //   Serial1 << remoteRC.rcCmd[i] << "\t";
  // Serial1 << speedDes << "\t" << yawDes << "\t" << vertDes << "\t" << remoteKnob << "\t";

  // Serial1 << remoteComputer.computerPacket.cmd << ": ";
  // for (int i=0; i<16; ++i) {
  //   Serial1 << remoteComputer.computerPacket.params[i] << " ";
  // }

  // // MOTORS ------------------------
  // for (int i=0; i<NMOT; ++i) {
  //   // UNCOMMENT THIS TO ZERO LEGS (Get raw pos when in jig; type into motZeros)
  //   Serial1 << _FLOAT(M[i].getRawPosition(), 3) << "\t";
  //   // Serial1 << M[i].getTorque() << "\t";
  // }

  // // LEG -----------------------------
  // for (int i=0; i<4; ++i) {
  //   // Serial1 << "[" << leg[i].getPosition(EXTENSION) << ","  << leg[i].getPosition(ANGLE) << "]\t";
  //   // leg forces
  //   Serial1 << ux[i] << "," << uz[i] << "\t";
  // }

  Serial1 << "\n";
}

void controlLoop() {
  uint32_t tic = micros();
  halUpdate();

  // BEHAVIOR
  // "soft start"
  if ((behavior == &bound || behavior == &walk) && softStart.running()) {
    float behavExtDes = (behavior == &bound) ? 1.5 : 1.0;
    softStart.update(behavExtDes);
  } else {
    behavior->update();
  }

  // Remote: set parameters, stop behavior
  remote->updateInterrupt();

  controlTime = micros() - tic;
}

void setup() {
  halInit();

  attachTimerInterrupt(0, controlLoop, CONTROL_RATE);
  // attachTimerInterrupt(1, debug, 20);//comment out when not needed to reduce interrupts

  if (remote != &remoteComputer)
    enable(true);
  // first behavior
  behavior->begin();

  // // test no remote
  // delay(10000);
  // behavior->begin();
  // digitalWrite(led1, LOW);
  // delay(5000);
  // behavior->end();
  // digitalWrite(led1, HIGH);
  // openLog.enable(true);
}

void loop() {
  // Behavior starting/signalling code, and nunchuck update go here
  remote->updateLoop();
}
