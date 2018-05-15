/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#include "SpinTail.h"

SpinTail spinTail;

float avg(float *myArray, int len){
  float sum = 0;
  float result;
  for (int i = 0; i < len; i++){
    sum += myArray[i];
  }
  result = sum/float(len);
  return result;
}

void SpinTail::begin() {
//  MinitaurLeg::useLengths = false;
    tStart = X.t;
    go = 1;
    Serial1 << tStart << "\n";
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
  if ((X.t-tStart)>1000) go = 0;
  M[8].setOpenLoop(0.2);
//  if (go == 1){
//      M[8].setOpenLoop(-0.1); //Open loop values!
//  }
//  else {
//    M[8].setOpenLoop(0); //safety thing happening
//  }
  //M[8].setGain(0.1);
  //M[8].setPosition(0);
  static float prevPos = M[8].getPosition();
  static int initialize = 1;
  static int prevTime = X.t;
  int dt;
  const int velocityBuffer = 300;
  static float velocities [velocityBuffer];
  static int counter = 0;
  static bool filledUpArray = 0;
  float pos;
  float normalizedPos;
  float DF;
  float tailVel;
  float dtheta;
  float instantVelocity;
  float betterVelocity;
  
  pos = M[8].getPosition();
  normalizedPos = pos;
  if (pos < prevPos) {
    normalizedPos = pos + 2*3.1415826;
  }
  DF = M[8].getOpenLoop();
  tailVel = M[8].getVelocity();
  
  if (initialize == 0){
    dtheta = normalizedPos - prevPos;
    if (dtheta < 0) Serial1 << "pos: " << pos << " prevPos: " << prevPos << "\n";
    dt = X.t-prevTime;
    instantVelocity = 1000.0 * dtheta/float(dt);
    velocities[counter] = instantVelocity;
    counter = (counter+1)%velocityBuffer;
    if (counter == velocityBuffer-1) filledUpArray = 1; //now we can average the velocities
    if (filledUpArray == 1){
      betterVelocity = avg(velocities, velocityBuffer);}
    else
      betterVelocity = instantVelocity;
    
    //Serial1 << "duty factor is " << DF << " and velocity is " << tailVel << "\n";
    //Serial1 << "position is at " << pos << "\n";
    //Serial1 << "getVelocity shows :" << tailVel << " and my velocity is : " << betterVelocity << "\n";
    //Serial1 << "dtheta: "<< dtheta << ", " << "dt: " << dt << " counter " << counter << " tStart : " << tStart << " Time: " << X.t << "\n";
    Serial1 << "Velocity: " << betterVelocity << " DF: " << DF << "\n";
    //Serial1.print(M[8].getOpenLoop());
    //  
    //  Serial1.print(time);    
    //  Serial1.print('\n');
  }
  else {
    initialize = 0;
  }
  prevTime = X.t;
  prevPos = pos;
}

