/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#include "IMUObject.h"


// convention: roll right > 0, pitch forward > 0
// corresponds to roll axis = +x (forward), pitch axis = +y (left)

// globals used by robot
volatile float rolldot = 0, pitchdot = 0, yawdot = 0;

// globals
IMUVN100 imuVN100;

void IMUVN100::begin() {
  vn100.init(PB12);
}

void IMUVN100::updateInterrupt() {
  float yaw=0, roll=0, pitch=0, _yawdot=0, _rolldot=0, _pitchdot=0;
  vn100.get(yaw, pitch, roll, _yawdot, _pitchdot, _rolldot);
  X.yaw = yaw;
  X.roll = pitch;
  X.pitch = roll;
  rolldot = _pitchdot;
  pitchdot = _rolldot;
  yawdot = _yawdot;
}

// ===== To save compile time, if using VN100, comment out everything below this ======

IMUMPU6000 imuMPU6000;

void IMUMPU6000::begin() {
  // delay for IMU
  delay(150);
  if (!mpu.init(PB12))
    errorStop("IMU comm failed");
  // Start off the EKF
  mpu.readSensors();
  ekf.init(mpu.acc);
}

void IMUMPU6000::updateInterrupt() {
  const float CONVERT_DERIVATIVES = 8;
  // IMU update
  mpu.readSensors();
  ekf.update(mpu.acc, mpu.gyr);
  const EulerState *e = ekf.getEuler();
  // put in IMU offsets here
  X.roll = e->angles[0] + 0.07;
  X.pitch = e->angles[1];
  X.yaw = e->angles[2];
  rolldot = e->angRates[0] * CONVERT_DERIVATIVES;
  pitchdot = e->angRates[1] * CONVERT_DERIVATIVES;
  yawdot = e->angRates[2];
}
