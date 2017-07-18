/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#include "FindSurface.h"
#include "Remote.h"      // Do this to access lastSignal from remote to set time to zero when switching behaviors

FindSurface findSurf;

void FindSurface::begin() {
  MinitaurLeg::useLengths = false;
}

void FindSurface::update() {
  
  uint32_t tstart = remoteRC.lastSignal;
  uint32_t t = X.t-tstart;
  
  float forceCutoff = 4.5;
  
  float angles[] = {0,0,0};
  float extensions[] = {1,2.6,1};
  float vels[] = {0,0,0};
  float times[] = {0,500,1000};
  int numpoints = ((sizeof times)/(sizeof times[0]));
  Interpolator interp = Interpolator(numpoints);
  float interpAng = interp.getSinglePVTInterp(angles, vels, times, t);
  float interpExt = interp.getSinglePVTInterp(extensions, vels, times, t);
  
  float standAng = 0,standExt = 1;
  float kExt = 0.1, kAng = 0.5;
  float ur, uth;
  uint32_t tEnableCutoff = 0.2*times[1];
  
  if (t <= tEnableCutoff){
    for (int i=0; i<4; ++i) {
      surfFound[i] = false;
    }
  }
  
  for(int i=0; i<4; ++i){
    leg[i].setGain(ANGLE, kAng);
    leg[i].setGain(EXTENSION, kExt);
  }
  
  for (int i=0; i<4; ++i) {
    float old_ur = ur;
    leg[i].getToeForce(ur, uth);

    if (surfFound[i]){
      leg[i].setPosition(ANGLE, standAng);
      leg[i].setPosition(EXTENSION, surfExt[i]);
      
    } else if (t >= tEnableCutoff && ((ur - old_ur) >= forceCutoff)){
      surfExt[i] = leg[i].getPosition(EXTENSION);
      leg[i].setPosition(ANGLE, standAng);
      leg[i].setPosition(EXTENSION, surfExt[i]);
      surfFound[i] = true;
      Serial1.print("Surface found on leg ");
      Serial1 << i << " at an extension of " << surfExt[i] << "\n";
      
    } else if (t <= times[numpoints-1]){
      leg[i].setPosition(ANGLE, interpAng);
      leg[i].setPosition(EXTENSION, interpExt);
      
    } else {
      leg[i].setPosition(ANGLE, standAng);
      leg[i].setPosition(EXTENSION, standExt);
    }
  }
}

