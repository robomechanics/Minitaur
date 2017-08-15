/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#ifndef Kick_h
#define Kick_h

#include "VirtualLeg.h"
#include <Behavior.h>

enum KickMode {
  KM_SIT = 0, KM_KICK
};

class Kick : public Behavior {
public:
  KickMode mode;
  bool bAbsLegAngle;//should remain false for MPU6000 use
  // "autopilot" sets desired speed by itself
  bool bAutopilot;
  uint32_t tstart;
  float headingDes;
  // bool waitingToStart;

  Kick() : bAbsLegAngle(false), bAutopilot(false), tstart(0), headingDes(0) {}
  // From base class
  void begin() {
    mode = KM_KICK;
  }
  void update();
  bool running() {
    // return false;
    return !(mode == KM_SIT);
  }
  void end() {
    mode = KM_SIT;
  }
  void signal() {
  }
};
extern Kick kick;

#endif

