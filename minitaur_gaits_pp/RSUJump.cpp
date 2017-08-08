/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Turner Topping
 */
#include "HAL.h"
#include "RSUJump.h"
#include "Remote.h"

RSUJump rsuJump;


/*
Minitaur legs (top view)  (right view)                (right view)   |\
 (front)                                            [[]]========[[]] | \
 0    2                     /    \                  [[]]========[[]]  )) - pitch  
 ||  ||                    /      \				   /  \        /  \  
 |XXXX|                    \      /				   \  /        \  /
 |XXXX| ---> +              \    /			        \/          \/
 |XXXX|                      \  /
 ||  ||                +  <-- \/ --> - 
 1    3
 (back)
Gameplan : 
*/

// float settLeft;
// float settRight;
// float timeEl;
// float cu0;
// float cu2;
//float highLong = vertDes;
void RSUJump::begin(){
	mode = RSUJ_WAIT; 
	tstart = X.t;
	ex0=0;
	an0=0;
	for(int i=0;i<4;++i){
		ex0=ex0+leg[i].getPosition(EXTENSION)/4.0;
		an0 = an0+leg[i].getPosition(ANGLE)/4.0;
	}	
}

void RSUJump::update() {
	MinitaurLeg::useLengths = false;

	float legAng = -X.pitch + 0.4;//map(vertDes, 0, 1, 0.4, 0.2);
	switch (mode) {
		default:
		case RSUJ_STAND:
			
			if(ex0>0.8){
				ex0 = ex0-.002;
			}else{
				ex0 = 0.8;
			}

			if(an0 > legAng+.01){
				an0 = an0 - .002;
			}else if(an0<legAng-.01){
				an0 = an0+.002;
			}else{
				an0 = legAng;
			}

			for (int i=0; i<4; ++i) {
				leg[i].setGain(ANGLE,.9,.002);
				leg[i].setGain(EXTENSION,.3,.002); //was 12
				leg[i].setPosition(ANGLE,an0);
				leg[i].setPosition(EXTENSION,ex0);
			}
			break;

		case RSUJ_WAIT:

			if(ex0>0.8){
				ex0 = ex0-.002;
			}else{
				ex0 = 0.8;
			}

			if(an0 > legAng+.05){
				an0 = an0 - .0005;
			}else if(an0<legAng-.05){
				an0 = an0+.0005;
			}else{
				an0 = legAng;
				mode = RSUJ_LEAP;
			}

			for (int i=0; i<4; ++i) {
				leg[i].setGain(ANGLE,.9,.002);
				leg[i].setGain(EXTENSION,.3,.002); //was 12
				leg[i].setPosition(ANGLE,an0);
				leg[i].setPosition(EXTENSION,ex0);
			}
			 
			break;

		case RSUJ_LEAP :
			leg[0].setOpenLoop(EXTENSION,1);
			leg[2].setOpenLoop(EXTENSION,1);

			leg[0].setPosition(ANGLE,legAng);
			leg[2].setPosition(ANGLE,legAng);
			X.xd = leg[1].getSpeed(X.pitch)/4.0+leg[3].getSpeed(X.pitch)/4.0+
			leg[0].getSpeed(X.pitch)/4.0 + leg[2].getSpeed(X.pitch)/4.0;
			//FRONT LEGS
			//leg[0].setPosition(EXTENSION,.7);
			//leg[2].setPosition(EXTENSION,.7);
			//leg[0].setPosition(ANGLE,-1.57);
			//leg[2].setPosition(ANGLE,-1.57);
			//HIND LEGS
			leg[1].setOpenLoop(EXTENSION,1);
			leg[3].setOpenLoop(EXTENSION,1);
			leg[1].setPosition(ANGLE,legAng);
			leg[3].setPosition(ANGLE,legAng);

			//TERMINATION
			if(leg[1].getPosition(EXTENSION)>2.8 || leg[3].getPosition(EXTENSION)>2.8){
				tarAng2 = leg[1].getPosition(ANGLE);
				mode = RSUJ_RETRACT;
				tEvent = millis();
				
			}
			break;

		case RSUJ_RETRACT :
			tarAng = -1.5;
			leg[0].setGain(ANGLE,.1);
			leg[2].setGain(ANGLE,.1);
			leg[0].setPosition(EXTENSION,0.8);
			leg[2].setPosition(EXTENSION,0.8);
			leg[1].setPosition(EXTENSION,0.6);
			leg[3].setPosition(EXTENSION,0.6);
			leg[1].setPosition(ANGLE,tarAng2);
			leg[3].setPosition(ANGLE,tarAng2);
			leg[0].setPosition(ANGLE,tarAng2);
			leg[2].setPosition(ANGLE,tarAng2);
			if(leg[1].getPosition(EXTENSION)<.64){
				tEvent = millis();
				mode = RSUJ_TUCKED; 
			}
		break;

		case RSUJ_TUCKED :
			leg[1].setPosition(EXTENSION,0.55);
			leg[3].setPosition(EXTENSION,0.55);
			leg[1].setPosition(ANGLE,-.8);
			leg[3].setPosition(ANGLE,-.8);
			leg[0].setPosition(EXTENSION,0.8);
			leg[2].setPosition(EXTENSION,0.8);
			leg[0].setPosition(ANGLE,.4);
			leg[2].setPosition(ANGLE,.4);
			// if(leg[0].getPosition(EXTENSION)<.6){
			// 	leg[0].setPosition(ANGLE,.4);
			// 	leg[2].setPosition(ANGLE,.4);
			// }

			if(leg[1].getPosition(ANGLE)<-.6){
				tEvent = millis();
				mode = RSUJ_AB;
				tarAng = -0.1;//-.7;
				tarAng2 = -0.1;//-.7;
				tarExt = .55;
				tarExt2 = .2;
			}
		break;


		 case RSUJ_AB :

		 	if(millis()-tEvent<75){
		 		//FRONT
		 		leg[0].setGain(ANGLE,.5);
				leg[2].setGain(ANGLE,.5);
		 		leg[0].setPosition(ANGLE,-.5-X.pitch);//-0.6-X.pitch);//-.7); // was -1.2
				leg[2].setPosition(ANGLE,-.5-X.pitch);//-0.6-X.pitch);//-.7);				
				//BACK
				leg[1].setPosition(EXTENSION,1.4);
				leg[3].setPosition(EXTENSION,1.4);

			}else if(millis()-tEvent>75 && millis()-tEvent<100){
				leg[0].setGain(ANGLE,.9);
				leg[2].setGain(ANGLE,.9);

				leg[0].setPosition(EXTENSION,.9);
				leg[2].setPosition(EXTENSION,.9);
		 		leg[0].setPosition(ANGLE,-.5-X.pitch);//-0.6-X.pitch);//-.7); // was -1.2
				leg[2].setPosition(ANGLE,-.5-X.pitch);//-0.6-X.pitch);//-.7);		
				// leg[0].setPosition(ANGLE,-0.7);//-.7);
				// leg[2].setPosition(ANGLE,-0.7);//-.7);
			}else if(millis()-tEvent>100){
				tEvent = millis();
				mode = RSUJ_LANDED;
				tddetf = false;
				tddetr = false;
				tarAng = -.8;
			}
			
		break;

		case RSUJ_LANDED :
			
			if(tddetf == true && tddetr == true){
				tEvent = millis();
				mode = RSUJ_POSTLANDED;
			}
			//Front LEG//////////////////
			// TD condition:
			if(leg[0].getPosition(EXTENSION)<.6 || leg[2].getPosition(EXTENSION)<.6){
				tddetf=true;
			}
			//PRIOR
			if(tddetf==false){
				leg[0].setPosition(EXTENSION,.9);
				leg[2].setPosition(EXTENSION,.9);

		 		leg[0].setPosition(ANGLE,-.5-X.pitch);//-0.6-X.pitch);//-.7); // was -1.2
				leg[2].setPosition(ANGLE,-.5-X.pitch);//-0.6-X.pitch);//-.7);		
			//POST
			}else{
				leg[0].setGain(EXTENSION,.6);
				leg[2].setGain(EXTENSION,.6);
				leg[0].setPosition(EXTENSION,1.3);
				leg[2].setPosition(EXTENSION,1.3);
				leg[0].setPosition(ANGLE,-.4);
				leg[2].setPosition(ANGLE,-.4);
			}	
			//FRONT LEG////////////////////
			//TD condition
			if(leg[1].getPosition(EXTENSION)<1.1 || leg[3].getPosition(EXTENSION)<1.1){
				tddetr = true;
				tEventRear = millis();
			}
			//PRIOR
			if(tddetr==false){
				leg[1].setPosition(EXTENSION,1.4);
				leg[3].setPosition(EXTENSION,1.4);
				leg[1].setPosition(ANGLE,-0.1);//-.7);
				leg[3].setPosition(ANGLE,-0.1);//-.7);
			//POST
			}else{
				if(tarAng>.2){
					tarAng = .2;
				}else{
					tarAng=tarAng+PI/400;
				}
				leg[1].setPosition(EXTENSION,1.4);
				leg[3].setPosition(EXTENSION,1.4);
				leg[1].setPosition(ANGLE,tarAng);
				leg[3].setPosition(ANGLE,tarAng);
				if(millis()-tEventRear>300){
					mode = RSUJ_POSTLANDED;
				}
			}


		break;

		case RSUJ_POSTLANDED:
	
			if(ex0>0.8){
				ex0 = ex0-.002;
			}else{
				ex0 = 0.8;
			}

			if(an0 > legAng+.01){
				an0 = an0 - .002;
			}else if(an0<legAng-.01){
				an0 = an0+.002;
			}else{
				an0 = legAng;
			}

			for (int i=0; i<4; ++i) {
				leg[i].setGain(ANGLE,.9,.002);
				leg[i].setGain(EXTENSION,.3,.002); //was 12
				leg[i].setPosition(ANGLE,an0);
				leg[i].setPosition(EXTENSION,ex0);
			}
			if(tEvent-millis()>200){
				mode = RSUJ_STAND;
			}
		break;

	}
}