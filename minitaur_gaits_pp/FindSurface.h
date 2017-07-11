/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#ifndef FindSurface_h
#define FindSurface_h

#include "VirtualLeg.h"
#include <Behavior.h>
#include "Interpolator.h"

class FindSurface : public Behavior {
public:
  // bool bAbsLegAngle;//should remain false for MPU6000 use
  // "autopilot" sets desired speed by itself
  // bool bAutopilot;
  // uint32_t tstart;
  // float headingDes;
  // bool waitingToStart;
  
  bool surfFound[4] = {};
  float surfExt[4] = {};
  float ur, uth;
  FindSurface(){}
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
extern FindSurface findSurf;

#endif
