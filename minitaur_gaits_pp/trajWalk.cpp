/**
Written by Joe Norby
 */
#include "trajWalk.h"

TrajWalk trajwalk;

void TrajWalk::begin() {
}

void TrajWalk::update() {
  MinitaurLeg::useLengths = false;
  int t = (X.t - remoteRC.lastSignal);
  float ext[4];
  float ang[4];

  // --------------- TUNABLES -------------------

  // Extension radius and neutral position
  float extRadius = 0.5;
  float extNom = 0.5*PI;

  // Angle radius and neutral position
  float angRadius = 0.2;
  float angNom = 0;

  // Phasing and period
  float phase0 = 0;  // 0 is in phase, 0.5 is anti-phase
  float phase1 = 0;
  float phase2 = 0;
  float phase3 = 0;
  int period = 1000; // in ms
  
  // Gains
  float kExt = 0.6;  // Don't exceed 1
  float kAng = 0.6;

  // --------------------------------------------

  ext[0] = -extRadius*arm_sin_f32((float)2*PI*t/(float)period + 2*PI*phase0) + extNom;
  ang[0] = angRadius*arm_cos_f32((float)2*PI*t/(float)period + 2*PI*phase0) + angNom;
  ext[1] = -extRadius*arm_sin_f32((float)2*PI*t/(float)period + 2*PI*phase1) + extNom;
  ang[1] = angRadius*arm_cos_f32((float)2*PI*t/(float)period + 2*PI*phase1) + angNom;
  ext[2] = -extRadius*arm_sin_f32((float)2*PI*t/(float)period + 2*PI*phase2) + extNom;
  ang[2] = angRadius*arm_cos_f32((float)2*PI*t/(float)period + 2*PI*phase2) + angNom;
  ext[3] = -extRadius*arm_sin_f32((float)2*PI*t/(float)period + 2*PI*phase3) + extNom;
  ang[3] = angRadius*arm_cos_f32((float)2*PI*t/(float)period + 2*PI*phase3) + angNom;
    
  for(int i=0; i<4; ++i){
    leg[i].setGain(ANGLE, kAng);
    leg[i].setGain(EXTENSION, kExt);
    leg[i].setPosition(EXTENSION, ext[i]);
    leg[i].setPosition(ANGLE, ang[i]);
  }
   
}


