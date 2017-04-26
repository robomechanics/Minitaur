/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#include "Flip.h"

Flip flip;

void Flip::update() {
  MinitaurLeg::useLengths = false;
  switch (mode) {
    default:
    case FLIP_STAND:
        for (int i=0; i<4; ++i) {
          leg[i].setGain(ANGLE,.7,.002);
  	  leg[i].setGain(EXTENSION,.4,.002); //was 12
  	  leg[i].setPosition(ANGLE, ((bInverted) ? -3.14 : 0)); 
  	  leg[i].setPosition(EXTENSION,((i==0||i==1) ? 1.57 : 1.57));
        }
      break;

    case FLIP_WAIT:
      for (int i=0; i<4; ++i) {
        leg[i].setGain(ANGLE,.7,.002);
	leg[i].setGain(EXTENSION,.4,.002); //was 12
	leg[i].setPosition(ANGLE, ((bInverted) ? -3.14 : 0)); 
	leg[i].setPosition(EXTENSION,((i==0||i==1) ? 1.0 : 1.57));
      }
      if (X.t - tstart > 100){
	mode = FLIP_GO;
	tEvent = millis();
      }
      break;

    case FLIP_GO:
      for (int i=0; i<4; ++i) {
        leg[i].setGain(ANGLE,.7,.002);
	leg[i].setGain(EXTENSION,((i==0||i==1) ? 1.0 : 0.2));
	leg[i].setPosition(ANGLE, ((bInverted) ? -3.14 : 0)); 
	leg[i].setPosition(EXTENSION,((i==0||i==1) ? 2.7 : 1));
      }
      if (X.t - tEvent > 225){
        mode = FLIP_STAND;
        tEvent = millis();
      }
      break;
      
    case FLIP_DONE:
      for (int i=0; i<4; ++i) {
        leg[i].setGain(ANGLE,.7,.002);
	leg[i].setGain(EXTENSION,.4,.002); //was 12
	leg[i].setPosition(ANGLE, ((bInverted) ? 3.14 : 0)); 
	leg[i].setPosition(EXTENSION,((i==0||i==1) ? 1.57 : 1.57));
      }
      break;
  }
}
