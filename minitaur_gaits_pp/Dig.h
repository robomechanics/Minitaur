/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#ifndef Dig_h
#define Dig_h

#include "VirtualLeg.h"
#include <Behavior.h>

class Dig : public Behavior {
public:
  bool bAbsLegAngle;//should remain false for MPU6000 use
  // "autopilot" sets desired speed by itself
  bool bAutopilot;
  uint32_t tstart;
  float headingDes;
  // bool waitingToStart;

  Dig() : bAbsLegAngle(false), bAutopilot(false), tstart(0), headingDes(0) {}
  // From base class
  void begin();
  void update();
  bool running() {
    return false;
  }
  void end() {
  }
  void signal() {
  }
};
extern Dig dig;

#endif

