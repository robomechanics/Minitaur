/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Turner Topping
 */
#ifndef RSUJump_h
#define RSUJump_h

// This behavior isn't declared to remote, just called from Walk, and return to walk

#include "Behavior.h"

enum RSUJumpMode {
  RSUJ_STAND, RSUJ_WAIT, RSUJ_FRONTHOP, RSUJ_PREJUMP, RSUJ_LEAP, 
  RSUJ_RETRACT, RSUJ_TUCKED, RSUJ_AB, RSUJ_LANDED
};

class RSUJump : public Behavior {
public: 
	RSUJump() : mode(RSUJ_STAND), tstart(0) {}

	void begin() {mode = RSUJ_WAIT; tstart = X.t;}
	void update();
	bool running() { return (mode!= RSUJ_STAND ); }
	void end() { if (mode == RSUJ_WAIT) mode = RSUJ_STAND;}
	void signal() {}

	RSUJumpMode mode;

  bool tddetf;
  bool tddetr;
  float tRet;
  float tEventRear;

  uint32_t tstart;
  float tEvent;
  float uroll;
  float tarAng;
  float tarAng2;
  float tarExt;
  float tarExt2;
  bool td= false;

  float tFHcomp;
  float fAngPos;
  float trackpullLeft;
  float trackpullRight;
  float tTakeOff;
  float tOnWall;
  float fExtPos;
  float cor;
};

extern RSUJump rsuJump;

#endif