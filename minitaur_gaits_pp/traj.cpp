/**
Written by Joe Norby
 */
#include "traj.h"

Traj traj;

void Traj::begin() {
}

void Traj::update() {
  MinitaurLeg::useLengths = false;
  float ang;
  float ext;
  int period = 5000;
  int t = (X.t - remoteRC.lastSignal) % period;
  
  float angles[] = {0,-0.5,-0.5,0.5,0.5,0};
  float extensions[] = {0.5,0.5,2,2,0.5,0.5};
  float vels[] = {0,0,0,0,0,0};
  float times[] = {0,1000,2000,3000,4000,5000};
  int numpoints = ((sizeof times)/(sizeof times[0]));
  Interpolator interp = Interpolator(numpoints);
  ang = interp.getSinglePVTInterp(angles, vels, times, t);
  ext = interp.getSinglePVTInterp(extensions, vels, times, t);
  
  // ang = interp.getSingleInterp(angles, times, t);
  // ext = interp.getSingleInterp(extensions, times, t);
  
  // ang = interp.getSingleZOH(angles, times, t);
  // ext = interp.getSingleZOH(extensions, times, t);

  
//  float standAng = 0,standExt = 1; 
  float kExt = 0.3, kAng = 0.3;
  
  for(int i=0; i<4; ++i){
    leg[i].setGain(ANGLE, kAng);
    leg[i].setGain(EXTENSION, kExt);
    leg[i].setPosition(ANGLE, 0);
    leg[i].setPosition(EXTENSION, 0.5*PI);
  }
  
  leg[0].setPosition(EXTENSION, ext);
  leg[0].setPosition(ANGLE, ang);
  
}


