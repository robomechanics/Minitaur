/**
Written by Joe Norby
 */
#ifndef trajWalk_h
#define trajWalk_h

#include "VirtualLeg.h"
#include "Interpolator.h"
#include <Behavior.h>
#include "Remote.h"

class TrajWalk : public Behavior {
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
extern TrajWalk trajwalk;

#endif
