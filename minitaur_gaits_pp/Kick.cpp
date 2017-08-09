/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#include "Kick.h"

Kick kick;

//void Kick::begin() {
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
//}

void Kick::update() {
  MinitaurLeg::useLengths = false; // just need it
  
  // is the time in miliseconds?
  // the update is called every KHz, was it?
  // what is tc?
  // what are the two for loops for?
  // if we put in extensions like 0 and pi, does it still accomodate for that?

  float standAng = 0, standExt = 1.57;
  float kExt = 0.4, kAng = 0.7;
  int tStartDelay = 50;
  int tStart = 450;
  int tInterval = 500;
  float pi = 3.14159265359;



  // float standAng = 0,standExt = 1.57; 
  // float kExt = 0.4, kAng = 0.7;
  // float tc = 3; // what is tc? Time constant?
  // int tReady = 1000/tc;
  // int tLower = 500/tc;
  // int tSweep = 2000/tc;
  // int tRaise = tLower;
  // int tReturn = tReady;
  // float sweep = -0.5;
  // float lift = -1.2;
  
  int time = X.t % (tStartDelay + tStart + tInterval); //to restart the kick when it ends

  //int time = X.t % (tLower+tReady+tSweep+tRaise+tReturn);
  

  //_____SIT MODE_________________________________________________
  if (mode == KM_SIT) {
    // for (int i=0; i<4; ++i) {
    //   leg[i].setGain(EXTENSION, 0.2);
    //   leg[i].setGain(ANGLE, 0.4, 0.01);
      
    //   leg[i].setPosition(EXTENSION, )
    // }
      
      
      //M[8].setGain(0.1,0.002);
      // M[8].setPosition(3.14/3); // THIS IS THE MAIN LINE
      M[8].setPosition(-X.pitch);
      
       //M[8].setOpenLoop(0.05);
    
      for(int i=0; i<4; ++i){
        leg[i].setGain(ANGLE, kAng+0.1);
        leg[i].setGain(EXTENSION, kExt);
      }
       leg[0].setGain(EXTENSION, kExt/4);

      for(int i=0; i<4; ++i){
        leg[i].setPosition(ANGLE, standAng);
        leg[i].setPosition(EXTENSION, standExt*(4/3));
      }
      leg[3].setPosition(EXTENSION, standExt/3);
      leg[0].setPosition(ANGLE, -(pi/2));
      leg[0].setPosition(EXTENSION, (3*pi/8));
      
      standExt = 2;  
      
      

      return;

  }












  
  
  // for(int i=0; i<4; ++i){
  //   leg[i].setGain(ANGLE, kAng+0.1);
  //   leg[i].setGain(EXTENSION, kExt);
  // }
  //  leg[0].setGain(EXTENSION, kExt/4);

  // for(int i=0; i<4;++i){
  //   leg[i].setPosition(ANGLE, standAng);
  //   leg[i].setPosition(EXTENSION, standExt*(4/3));
  // }
  // leg[3].setPosition(EXTENSION, standExt/3);
  
  // standExt = 2;  
  


  //M[8].setOpenLoop(0.4);
  // M[8].setPosition(-3.14/3); // THIS IS THE  MAIN LINE

  M[8].setPosition(-pi/4);


  if (time<tStartDelay) {
    leg[0].setPosition(ANGLE, -(pi/2));
    leg[0].setPosition(EXTENSION, (3*pi/8));
    // leg[2].setPosition(ANGLE, -(pi/2));
    // leg[2].setPosition(EXTENSION, (7*pi/8));
  } 
  else if ((tStartDelay <= time) && (time < tStartDelay+tStart)) {
    leg[0].setPosition(ANGLE, -(pi/2));
    leg[0].setPosition(EXTENSION, (3*pi/8));
    // leg[2].setPosition(ANGLE, -(pi/2));
    // leg[2].setPosition(EXTENSION, (7*pi/8));
  }
  else { //if ((time >= tStart) && (time < tStart + tInterval)) {
    leg[0].setPosition(ANGLE, -(pi/2));
    leg[0].setPosition(EXTENSION, (7*pi/8));
    // leg[2].setPosition(ANGLE, -(pi/2));
    // leg[2].setPosition(EXTENSION, (3*pi/8));
  }








  // if (time<tReady){
  //     leg[0].setPosition(ANGLE, standAng + (float)time*(sweep)/(float)(tReady-0));
  //     leg[0].setPosition(EXTENSION, standExt + lift);
  // }
  // else if(time>=(tReady) && time<(tReady+tLower)){
  //     leg[0].setPosition(ANGLE, standAng + sweep);
  //     leg[0].setPosition(EXTENSION, (standExt+lift) + (float)(time-tReady)*(-lift)/(float)(tLower));
  // }
  // else if(time>=(tReady+tLower) && time<(tReady+tLower+tSweep)){
  //     leg[0].setPosition(ANGLE, (standAng+sweep) + (float)(time-(tReady+tLower))*(-2*sweep)/(float)(tSweep));
  //     leg[0].setPosition(EXTENSION, standExt);
  // }
  // else if(time>=(tReady+tLower+tSweep) && time<(tReady+tLower+tSweep+tRaise)){
  //     leg[0].setPosition(ANGLE, standAng - sweep);
  //     leg[0].setPosition(EXTENSION, (standExt) + (float)(time-(tReady+tLower+tSweep))*(lift)/(float)(tRaise));
  // }
  // else{
  //     leg[0].setPosition(ANGLE, (standAng - sweep) + (float)(time-(tReady+tLower+tSweep+tRaise))*(sweep)/(float)(tReturn));
  //     leg[0].setPosition(EXTENSION, standExt + lift); 
  // }




//  
// Serial1.print(time);    
//  Serial1.print('\n');

  //for (int i=0; i<4; ++i) {
  //   Serial1 << "[" << leg[i].getPosition(EXTENSION) << ","  << leg[i].getPosition(ANGLE) << "]\t";
     // leg forces
     // Serial1 << ux[i] << "," << uz[i] << "\t";
  // }


}


