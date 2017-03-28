#ifndef Walk_h
#define Walk_h

#include "VirtualLeg.h"
#include <Behavior.h>



enum WalkMode {
  WM_SIT, WM_WAIT, WM_WALK
};

class Walk : public Behavior {
public:
  WalkMode mode;
  uint32_t tstart;
  // float phase;
  // float clockedLegPhase[4];
  int flightLeg, nextFlightLeg, stepLeg, nextStepLeg;
  float pep;
  uint32_t tLO, tTD;
  float absAngles[4];
  uint32_t lastUpdate;
  float frac;//fraction through flight, 0 if in stance
  SignalState signalState;

  Walk() : mode(WM_SIT), tstart(0), lastUpdate(0), signalState(SIGNAL_NONE) {
    init();
  }
//  WalkLeg wl[4];

//  Walk() : wl({WalkLeg(0), WalkLeg(1), WalkLeg(2), WalkLeg(3)}) {}
  void init() {
    flightLeg = -1;
    nextFlightLeg = 1;//3;
    stepLeg = -1;
    nextStepLeg = -1;
    for (int i=0; i<4; ++i)
      absAngles[i] = 0;
    tLO = 0;
    tTD = 0;
  }
  // From base class
  void begin() {
    MinitaurLeg::useLengths = false;
    mode = WM_WAIT;
    tstart = X.t;
  }
  void update();
  bool running() {
    // return !(d03.mode == STAND && d21.mode == STAND);
    return !(mode == WM_SIT);
  }
  void end() {
    mode = WM_SIT;
  }
  void signal() {
    // FIXME for now use this for step over, later replace with proprio
    signalState = SIGNAL_QUEUE;
  }
};
extern Walk walk;

#endif
