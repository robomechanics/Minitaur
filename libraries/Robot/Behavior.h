/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#ifndef Behavior_h
#define Behavior_h

#include <stdint.h>

/** @addtogroup Behavior
 *  @{
 */

/**
 * @brief Abstract base class for implementing behaviors
 */
class Behavior {
public:
  virtual void begin() = 0;
  virtual void update() = 0;
  virtual bool running() = 0;
  virtual void end() = 0;
  virtual void signal(uint8_t sig=0) {}
};

/** @} */ // end of addtogroup

#endif