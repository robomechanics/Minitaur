/**
Written by Joe Norby
 */
#include "traj.h"

Traj traj;

void Traj::begin() {
}

void Traj::update() {
  MinitaurLeg::useLengths = false;
  int t = X.t % 5000;
  
  float angles[] = {0,-0.5,-0.5,0.5,0.5,0};
  float extensions[] = {0.5,0.5,2,2,0.5,0.5};
  float vels[] = {0,0,0,0,0,0};
  float times[] = {0,1000,2000,3000,4000,5000};
  int numpoints = ((sizeof times)/(sizeof times[0]));
  Interpolator interp = Interpolator(numpoints);
  float ang = interp.getSinglePVTInterp(angles, vels, times, t);
  float ext = interp.getSinglePVTInterp(extensions, vels, times, t);

  
//  float standAng = 0,standExt = 1; 
  float kExt = 0.3, kAng = 0.3;
  
  for(int i=0; i<4; ++i){
    leg[i].setGain(ANGLE, kAng);
    leg[i].setGain(EXTENSION, kExt);
  }
  
  leg[0].setPosition(EXTENSION, ext);
  leg[0].setPosition(ANGLE, ang);
  
}


