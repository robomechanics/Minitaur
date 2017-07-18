#ifndef FrontFlip_h
#define FrontFlip_h

#include "Behavior.h"
enum FFMode { 
	FF_STAND, FF_WAIT, FF_JUMP, FF_INVERT, FF_LAND, FF_POSTLAND
	};
class FrontFlip : public Behavior {
public: 
	FrontFlip() : mode(FF_STAND), tstart(0) {}

	void begin();
	void update();
	bool running() {return !(mode == FF_STAND || mode == FF_POSTLAND);}
	void end() {}//flipping = false;}
	void signal() {}

	FFMode mode;
	float g00s3 = 1;
	float standStiff = .5;
	float startingHeight = 1.04; //.13m now in De-dians!
	float jumpStiff = .3;
	float landingHeight = 1.54; //.17 meters
	float landingStiff = .5;
	uint32_t tstart;
	float tLandStart;
};

extern FrontFlip frontFlip;

#endif