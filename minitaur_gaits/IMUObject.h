/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#ifndef IMUObject_h
#define IMUObject_h

#include "HAL.h"
#include "Peripheral.h"

extern Peripheral *imu;

#include <VN100.h>
class IMUVN100 : public Peripheral {
public:
  VN100 vn100;
  uint8_t errId;

  IMUVN100() : vn100(SPI_2), errId(0) {}
  // called from setup()
  virtual void begin();
  // called from controlLoop(), don't use delay()
  virtual void updateInterrupt();
  // called from loop(): can use delay()
  virtual void updateLoop() {}
};
extern IMUVN100 imuVN100;

// ===== To save compile time, if using VN100, comment out everything below this ======

// #include <MPU6000.h>
// #include <EKF.h>
// class IMUMPU6000 : public Peripheral {
// public:
//   MPU6000 mpu;
//   EKF ekf;

//   IMUMPU6000() : mpu(SPI_2), ekf(0.1, 1.0, 1/((float)CONTROL_RATE)) {}
//   // called from setup()
//   virtual void begin();
//   // called from controlLoop(), don't use delay()
//   virtual void updateInterrupt();
//   // called from loop(): can use delay()
//   virtual void updateLoop() {}
// };
// extern IMUMPU6000 imuMPU6000;

// ============================================

#endif
