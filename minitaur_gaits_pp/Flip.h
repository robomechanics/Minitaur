/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#ifndef Flip_h
#define Flip_h

#include "VirtualLeg.h"
#include "ReorientableBehavior.h"

enum FlipMode {
  FLIP_STAND, FLIP_WAIT, FLIP_GO, FLIP_DONE
};

class Flip : public ReorientableBehavior {
public:
  Flip() : mode(FLIP_STAND), tstart(0) {}

  void begin() {mode = FLIP_WAIT; tstart = X.t;}
  void update();
  bool running() { return (mode!= FLIP_STAND ); }
  void end() { if (mode == FLIP_DONE) mode = FLIP_STAND;}
  void signal() {}

  FlipMode mode;

  uint32_t tstart;
  float tEvent;
};
extern Flip flip;

#endif
