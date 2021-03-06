/**
Written by Joe Norby
 */
#ifndef traj_h
#define traj_h

#include "VirtualLeg.h"
#include "Interpolator.h"
#include <Behavior.h>
#include "Remote.h"

class Traj : public Behavior {
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
extern Traj traj;

#endif
