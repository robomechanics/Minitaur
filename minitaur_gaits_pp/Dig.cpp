/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#include "Dig.h"

Dig dig;

void Dig::begin() {
//  MinitaurLeg::useLengths = false;
//  tstart = X.t;
//  
//  for (int i=0; i<4; ++i){
//    Serial1.print(leg[i].getPosition(ANGLE),3);
//    Serial1.print('\t');
//    Serial1.print(leg[i].getPosition(EXTENSION),3);
//    Serial1.print('\t');
//  }
//  Serial1.print('\n');
//  
//  float kExt = 0.4;
//  float kAng = 0.4;
//  float standAng = 0;
//  float standExt = 1.57;
//
//  for(int i=0; i<4; ++i){
//    leg[i].setGain(ANGLE, kAng);
//    leg[i].setGain(EXTENSION, kExt);
//  }
//  leg[3].setGain(EXTENSION, kExt/4);
//  for(int i=0; i<4;++i){
//    leg[i].setPosition(ANGLE, standAng);
//    leg[i].setPosition(EXTENSION, standExt);
//  }
//  
}

void Dig::update() {
  MinitaurLeg::useLengths = false;
  
  float standAng = 0,standExt = 1.57; 
  float kExt = 0.4, kAng = 0.7;
  float tc = 3;
  int tReady = 1000/tc;
  int tLower = 500/tc;
  int tSweep = 2000/tc;
  int tRaise = tLower;
  int tReturn = tReady;
  float sweep = -0.5;
  float lift = -1.2;
  
  int time = X.t % (tLower+tReady+tSweep+tRaise+tReturn);
  
  for(int i=0; i<4; ++i){
    leg[i].setGain(ANGLE, kAng+0.1);
    leg[i].setGain(EXTENSION, kExt);
  }
  leg[2].setGain(EXTENSION, kExt/4);

  for(int i=0; i<4;++i){
    leg[i].setPosition(ANGLE, standAng);
    leg[i].setPosition(EXTENSION, standExt);
  }
  leg[1].setPosition(EXTENSION, standExt/2);
  
  standExt = 2;  
  
  if (time<tReady){
      leg[2].setPosition(ANGLE, standAng + (float)time*(sweep)/(float)(tReady-0));
      leg[2].setPosition(EXTENSION, standExt + lift);
  }
  else if(time>=(tReady) && time<(tReady+tLower)){
      leg[2].setPosition(ANGLE, standAng + sweep);
      leg[2].setPosition(EXTENSION, (standExt+lift) + (float)(time-tReady)*(-lift)/(float)(tLower));
  }
  else if(time>=(tReady+tLower) && time<(tReady+tLower+tSweep)){
      leg[2].setPosition(ANGLE, (standAng+sweep) + (float)(time-(tReady+tLower))*(-2*sweep)/(float)(tSweep));
      leg[2].setPosition(EXTENSION, standExt);
  }
  else if(time>=(tReady+tLower+tSweep) && time<(tReady+tLower+tSweep+tRaise)){
      leg[2].setPosition(ANGLE, standAng - sweep);
      leg[2].setPosition(EXTENSION, (standExt) + (float)(time-(tReady+tLower+tSweep))*(lift)/(float)(tRaise));
  }
  else{
      leg[2].setPosition(ANGLE, (standAng - sweep) + (float)(time-(tReady+tLower+tSweep+tRaise))*(sweep)/(float)(tReturn));
      leg[2].setPosition(EXTENSION, standExt + lift); 
  }
//  
//  Serial1.print(time);    
//  Serial1.print('\n');

}



