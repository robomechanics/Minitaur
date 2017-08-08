/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */
#ifndef IMUReset_h
#define IMUReset_h

#include "Behavior.h"
#include "IMUObject.h"

class IMUReset : public Behavior {
public:
	IMUReset() {}

	void begin() {
    imuVN100.vn100.reset();
    delay(1000);
  }
	void update() {}
	bool running() {return false;}
	void end() {}
	void signal() {}
};

extern IMUReset imuReset;

#endif