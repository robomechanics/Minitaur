/**
Written by Joe Norby
 */
#include "forcefeedback.h"

Force force;

void Force::begin() {
}

void Force::update() {
  MinitaurLeg::useLengths = false;
  
//  float standAng = 0,standExt = 1; 
  float kExt = 0.5, kAng = 0.5;
  
  for(int i=0; i<4; ++i){
    leg[i].setGain(ANGLE, kAng);
    leg[i].setGain(EXTENSION, kExt);
  }

  for(int i=0; i<2;++i){
    leg[i+2].setPosition(EXTENSION, leg[i].getPosition(EXTENSION));
    leg[i+2].setPosition(ANGLE, leg[i].getPosition(ANGLE));
  }
  
  for(int i=0; i<2;++i){
    leg[i].setPosition(EXTENSION, leg[i+2].getPosition(EXTENSION));
    leg[i].setPosition(ANGLE, leg[i+2].getPosition(ANGLE));
  }

}


