/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#ifndef Bound_h
#define Bound_h

#include "VirtualLeg.h"
#include <Behavior.h>

class BoundLeg : public LegPair {
public:
  // BOUND
  // returns true when beginning leap stance
  bool update(bool bAbsLegAngle, bool bAutopilot, uint32_t tstart, float headingDes);
  // 0 = normal
  // 1 = (waiting for) stance -- set by Bound::signal()
  // 2 = landing -- in the flight after the leap stance
  volatile SignalState signalState;
  float phaseControlGain;

  BoundLeg(int i1, int i2) : LegPair(i1, i2), signalState(SIGNAL_NONE), phaseControlGain(0) {}
};

class Bound : public Behavior {
public:
  BoundLeg front, rear;
  bool bAbsLegAngle;//should remain false for MPU6000 use
  // "autopilot" sets desired speed by itself
  bool bAutopilot;
  uint32_t tstart;
  float headingDes;
  // bool waitingToStart;

  Bound() : front(BoundLeg(0, 2)), rear(BoundLeg(1, 3)), bAbsLegAngle(false), bAutopilot(false), tstart(0), headingDes(0) {}
  // From base class
  void begin();
  void update() {
    bool frontLeaping = front.update(bAbsLegAngle, bAutopilot, tstart, headingDes);
    if (frontLeaping)
      rear.signalState = SIGNAL_QUEUE;
    rear.update(bAbsLegAngle, bAutopilot, tstart, headingDes);
    // Position reading errors can cause catastrophic large changes in xd
    float newSpeed = 0.5 * (front.speed + rear.speed);
    if (fabsf(newSpeed - X.xd) < 0.2)
      X.xd = newSpeed;
    X.mode = (uint8_t)front.mode + (uint8_t)((rear.mode)<<2);
  }
  bool running() {
    return !(front.mode == STAND && rear.mode == STAND);
  }
  void end() {
    front.end();
    rear.end();
  }
  void signal() {
    front.signalState = SIGNAL_QUEUE;
    // front will start rear -- this is in update()
    // wait till front is in flight
    // while (front.mode == FLIGHT) {}
    // front.signalState = SIGNAL_LEAP_STANCE;
    // // wait for first jump to finish
    // while (front.signalState == SIGNAL_LEAP_STANCE) {}
    // rear.signalState = SIGNAL_LEAP_STANCE;
  }
  void useAbsLegAngle(bool flag) { bAbsLegAngle = flag; }
  void autopilot(bool flag) { bAutopilot = flag; }
  void setPhaseControlGain(float gain) {
    front.phaseControlGain = gain;
    rear.phaseControlGain = -gain;
  }
};
extern Bound bound;

#endif