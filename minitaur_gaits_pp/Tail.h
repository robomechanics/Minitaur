/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#ifndef Tail_h
#define Tail_h

#include "VirtualLeg.h"
#include <Behavior.h>

enum TailMode {
  TM_SIT = 0, TM_KICK
};

class Tail : public Behavior {
public:
  TailMode mode;
  bool bAbsLegAngle;//should remain false for MPU6000 use
  // "autopilot" sets desired speed by itself
  bool bAutopilot;
  uint32_t tstart;
  float headingDes;
  // bool waitingToStart;

  Tail() : bAbsLegAngle(false), bAutopilot(false), tstart(0), headingDes(0) {}
  // From base class
  void begin() {
    mode = TM_KICK;
  }
  void update();
  bool running() {
    // return false;
    return !(mode == TM_SIT);
  }
  void end() {
    mode = TM_SIT;
  }
  void signal() {
  }
};
extern Tail tail;

#endif

