/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 * 
 * You are free to use this gait for non-commercial purposes. If you use it in
 * research, please cite 
 * - A. De and D. E. Koditschek, “Vertical hopper compositions for preflexive and feedback-stabilized quadrupedal bounding, pronking, pacing and trotting,” under review, Sep. 2016.
 * 
 */
#ifndef Bound_h
#define Bound_h

#include "VirtualLeg.h"
#include "ReorientableBehavior.h"
#include <VN100.h>

class Bound;

class BoundLeg : public LegPair {
  Bound *pBound;
  bool waitForTD;
public:
  // BOUND
  // returns true when beginning leap stance
  bool update();
  // 0 = normal
  // 1 = (waiting for) stance -- set by Bound::signal()
  // 2 = landing -- in the flight after the leap stance
  volatile SignalState signalState;

  BoundLeg(Bound *pBound, int i1, int i2) : LegPair(i1, i2), pBound(pBound), waitForTD(false), signalState(SIGNAL_NONE) {}
};

// ------------------------------

// Parameters struct
// Different per robot
struct BoundParameters {
  float angNom[2]; // front/rear
  // hybrid system params
  // tstance is now only used for the FA kStrideLength, nothing else
  uint32_t tstance;
  float tdExt;
  // stance virtual spring params
  float kSpring, kVert, kOffset;
  // FA params
  float kAngStance;
  float kExtFltP, kExtFltD, kAngFltP, kAngFltD;
  float kNomLegRad;
  // Attitude control params
  float kRoll, kRollDot;
  // Retraction params
  uint32_t tretract;
  float extMin;
  // Stand params
  float kExtStand, kAngStand, kAngStandD;
};

class Bound : public ReorientableBehavior {
public:
  BoundLeg front, rear;
  bool bAbsLegAngle;//should remain false for MPU6000 use
  // "autopilot" sets desired speed by itself
  bool bAutopilot;
  uint32_t tstart;
  float headingDes;
  float phaseControlGain;
  const BoundParameters *params;
  float kStrideLength, flightPos, speedErr;

  Bound() : front(BoundLeg(this, 0, 2)), rear(BoundLeg(this, 1, 3)), bAbsLegAngle(false), bAutopilot(false), tstart(0), headingDes(0), phaseControlGain(0) {}
  // From base class
  void begin();
  void update() {
    MinitaurLeg::useLengths = false;
    if (isReorienting())
      return;
    // variables for both legs
    this->speedErr = constrain(speedDes-X.xd, -0.05, 0.05);

    // FIXME these should access pBound and set some flag instead of this return value
    bool frontLeaping = front.update();
    if (frontLeaping && rear.signalState == SIGNAL_NONE)
      rear.signalState = SIGNAL_QUEUE;
    rear.update();
    // low pass by averaging from and rear
    float newSpeed = 0.5 * (front.speed + rear.speed);
    // if (fabsf(newSpeed - X.xd) < 0.2)
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
  void signal(uint8_t sig) {
    front.signalState = SIGNAL_QUEUE;
    // front will start rear -- this is in update()
    // wait till front is in flight
    // while (front.mode == FLIGHT) {}
    // front.signalState = SIGNAL_LEAP_STANCE;
    // // wait for first jump to finish
    // while (front.signalState == SIGNAL_LEAP_STANCE) {}
    // rear.signalState = SIGNAL_LEAP_STANCE;
  }
};
extern Bound bound;


#endif