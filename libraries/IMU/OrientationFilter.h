/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#ifndef OrientationFilter_h
#define OrientationFilter_h

/** @addtogroup IMU
 *  @{
 */

typedef struct {
  float angles[3];
  float angRates[3];
} EulerState;


/**
 * @brief Base class for attitude filters
 */
class OrientationFilter {
public:
  /**
   * @brief Update filter from measurements (usually from IMU.readSensors())
   * @param acc Measured acc
   * @param gyr Measured gyr
   */
  virtual void update(const float *acc, const float *gyr) = 0;

  /**
   * @brief Get filtered Euler angles
   * @return `e->angles[]` is a `float[3]` array of angles and, `e->angRates[]` of angular rates
   */
  virtual const EulerState *getEuler() = 0;
  
protected:
  EulerState euler;
};

/** @} */ // end of addtogroup

#endif
