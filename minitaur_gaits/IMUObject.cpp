/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#include "IMUObject.h"


// convention: roll right > 0, pitch forward > 0
// corresponds to roll axis = +x (forward), pitch axis = +y (left)

// globals
IMUVN100 imuVN100;

void IMUVN100::begin() {
  vn100.init(PB12);
}

void IMUVN100::updateInterrupt() {
  float yaw=0, roll=0, pitch=0, _yawdot=0, _rolldot=0, _pitchdot=0;
  vn100.get(yaw, pitch, roll, _yawdot, _pitchdot, _rolldot);
  // make sure all of the response header is what is expected. weird noise issue upside down
  if (vn100.resphead[0] == 0 && vn100.resphead[1] == 1 && vn100.resphead[2] == 240 && vn100.resphead[3] == 0) {
    X.yaw = yaw;
    X.roll = pitch;
    X.pitch = roll;
    X.rolldot = _pitchdot;
    X.pitchdot = _rolldot;
    X.yawdot = _yawdot;
  }
}

// ===== To save compile time, if using VN100, comment out everything below this ======

// IMUMPU6000 imuMPU6000;

// void IMUMPU6000::begin() {
//   // delay for IMU
//   delay(150);
//   if (!mpu.init(PB12))
//     errorStop("IMU comm failed");
//   // Start off the EKF
//   mpu.readSensors();
//   ekf.init(mpu.acc);
// }

// void IMUMPU6000::updateInterrupt() {
//   const float CONVERT_DERIVATIVES = 8;
//   // IMU update
//   mpu.readSensors();
//   ekf.update(mpu.acc, mpu.gyr);
//   const EulerState *e = ekf.getEuler();
//   // put in IMU offsets here
//   X.roll = e->angles[0];
//   X.pitch = e->angles[1];
//   X.yaw = e->angles[2];
//   // handles angles differently than VN100 when upside down
//   // VN100 pitch [-pi,pi], roll [-pi/2,pi]
//   // - when rolled, roll goes up to pi/2 then back down to 0
//   // - when pitched, pitch goes to pi then jumps to -pi
//   // MPU6000 angles pitch [-pi/2,pi/2], roll [-pi,pi]
//   // - when pitched, pitch goes up to pi/2 then back down to 0
//   // - when rolled, roll goes to pi then jumps to -pi
//   // convert MPU6000 standard to VN100
//   if (fabsf(X.roll) > HALF_PI) {
//     X.roll = (X.roll > 0) ? PI - X.roll : -PI - X.roll;
//     X.pitch = (X.pitch > 0) ? PI - X.pitch : -PI - X.pitch;
//   }
//   rolldot = e->angRates[0] * CONVERT_DERIVATIVES;
//   pitchdot = e->angRates[1] * CONVERT_DERIVATIVES;
//   yawdot = e->angRates[2];
// }
