/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#ifndef Peripheral_h
#define Peripheral_h

// base class
class Peripheral {
public:
  // called from setup()
  virtual void begin() = 0;
  // called from controlLoop(), don't use delay()
  virtual void updateInterrupt() = 0;
  // called from loop(): can use delay()
  virtual void updateLoop() = 0;
};

#endif
