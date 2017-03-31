/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#ifndef Walk_h
#define Walk_h

#include "VirtualLeg.h"
#include "ReorientableBehavior.h"

enum WalkMode {
  WM_SIT = 0, /*WM_WAIT, */WM_WALK, WM_REORIENT
};

class Walk : public ReorientableBehavior {
public:
  WalkMode mode;
  // Some timestamps to keep track of
  uint32_t tstart;
  uint32_t lastUpdate;
  uint32_t tLO, tTD;
  // State for taking steps
  int flightLeg, nextFlightLeg, stepLeg, nextStepLeg;
  // LO PEP (state to decide touchdown angle)
  float pep;
  // Desired nominal leg angles for each leg
  float absAngles[4];
  //fraction through flight, 0 if in stance
  float frac;
  DLPF speedFilt;
  // // Old clocked
  // float phase;
  // float clockedLegPhase[4];

  SignalState signalState;

  Walk() : mode(WM_SIT), tstart(0), lastUpdate(0), signalState(SIGNAL_NONE) {
    init();
    speedFilt.init(0.999, CONTROL_RATE, DLPF_SMOOTH);
  }

  /**
   * @brief This function resets some of the state related to walking, like which leg is 
   * in flight etc.
   */
  void init() {
    flightLeg = -1;
    nextFlightLeg = 1;//3;

    // for stepping - ignore
    stepLeg = -1;
    nextStepLeg = -1;
    //
    for (int i=0; i<4; ++i)
      absAngles[i] = 0;
    tLO = 0;
    tTD = 0;
  }
  // From base class
  void begin() {
    MinitaurLeg::useLengths = false;
    mode = WM_WALK;//WM_WAIT;
    tstart = X.t;
  }
  void update();
  bool running() {
    return !(mode == WM_SIT);
  }
  void end() {
    mode = WM_SIT;
  }
  void signal(uint8_t sig);
};
extern Walk walk;

#endif
