/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#include "SpinTail.h"

SpinTail spinTail;

void SpinTail::begin() {
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

void SpinTail::update() {
  MinitaurLeg::useLengths = false;
  //M[8].setOpenLoop(0.01);
  M[8].setGain(0.1);
  M[8].setPosition(0);
  Serial1.print(M[8].getOpenLoop());
  Serial1.print("  \n");
  Serial1.print(M[8].getRawPosition());
//  
//  Serial1.print(time);    
//  Serial1.print('\n');

}

