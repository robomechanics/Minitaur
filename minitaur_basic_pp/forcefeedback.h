/**
Written by Joe Norby
 */
#ifndef Force_h
#define Force_h

#include "VirtualLeg.h"
#include <Behavior.h>

class Force : public Behavior {
public:

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
extern Force force;

#endif
