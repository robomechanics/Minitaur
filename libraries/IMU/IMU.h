/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#ifndef IMU_h
#define IMU_h

#include <stdint.h>
#include <math.h>

/** @addtogroup IMU IMU drivers and attitude estimation
 *  @{
 */

/**
 * @brief Base IMU class that any IMU hardware should derive from
 */
class IMU {
public:
  /**
   * @brief Read IMU hardware and set acc, gyr in units of m/s^2 and rad/s
   * @details This function must be implemented by derived classes
   */
  virtual void readSensors() = 0;

  float getZ() {
    return acc[2];
  }

  float acc[3];
  float gyr[3];
protected:
  // OrientationFilter *filt;
};

/** @} */ // end of addtogroup

#endif
