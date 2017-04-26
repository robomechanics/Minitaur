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


void RSUJump::update() {
	MinitaurLeg::useLengths = false;
	switch (mode) {
		default:
		case RSUJ_STAND:
			for (int i=0; i<4; ++i) {
				leg[i].setGain(ANGLE,.7,.002);
				leg[i].setGain(EXTENSION,.3,.002); //was 12
				leg[i].setPosition(ANGLE, ((i==0||i==2) ? .3:.5)); 
				leg[i].setPosition(EXTENSION,((i==0||i==2)?.6:.6));
			}
			break;

		case RSUJ_WAIT:
			for (int i=0; i<4; ++i) {
				leg[i].setGain(ANGLE,.7,.002);
				leg[i].setGain(EXTENSION,.3,.002); //was 12
				leg[i].setPosition(ANGLE, ((i==0||i==2) ? .3:.5)); 
				leg[i].setPosition(EXTENSION,((i==0||i==2)?.6:.6));
			}
			if (X.t - tstart > 200)
				mode = RSUJ_LEAP;
				tEvent = millis();
			break;

		case RSUJ_FRONTHOP :
			// leg[0].setOpenLoop(EXTENSION, 1);
			// leg[2].setOpenLoop(EXTENSION, 1);//Open Loop with Hind Legs
			// leg[1].setPosition(ANGLE,-X.pitch+.65); //
			// leg[3].setPosition(ANGLE,-X.pitch+.65); //Back and Away
			// leg[0].setPosition(ANGLE,0); // Maintain leg angle while 
			// leg[2].setPosition(ANGLE,0); // loading the body
			// leg[1].setPosition(EXTENSION,1.1);
			// leg[3].setPosition(EXTENSION,1.1);
			// if(leg[0].getPosition(EXTENSION) > 2.7|| leg[2].getPosition(EXTENSION) > 2.7 ){
			// 	//Once legs have extended, end jump
			// 	mode = RSUJ_PREJUMP;
			// 	tarAng2 = (leg[0].getPosition(ANGLE)+leg[2].getPosition(ANGLE))/2-.3;
			// 	//leg[0].setPosition(EXTENSION,.2);
			// 	//leg[2].setPosition(EXTENSION,.2);
			// 	tEvent = millis();
			// 	//fAngPos = -.1;
			// 	//fExtPos = .15;
			// 	tarAng = .3;
			// }
			break;


		case RSUJ_PREJUMP :
			// //Retrect Front legs
			// leg[0].setPosition(EXTENSION,.7);
			// leg[2].setPosition(EXTENSION,.7);
			// leg[0].setPosition(ANGLE,-X.pitch); 
			// leg[2].setPosition(ANGLE,-X.pitch);
			// leg[1].setPosition(ANGLE,-X.pitch+.65);
			// leg[3].setPosition(ANGLE,-X.pitch+.65);

			// if(fabsf(X.pitch) > .7){ // PI/3
			// 	//At some angle that matters,
			// 	//start the front leap. 
			// 	mode = RSUJ_LEAP;
			// 	tEvent = millis(); //TimeKeeping! (do the little things)
			// }
			break;

		case RSUJ_LEAP :
			leg[0].setOpenLoop(EXTENSION,1);
			leg[2].setOpenLoop(EXTENSION,1);

			leg[0].setPosition(ANGLE,.5);
			leg[2].setPosition(ANGLE,.5);
			//FRONT LEGS
			//leg[0].setPosition(EXTENSION,.7);
			//leg[2].setPosition(EXTENSION,.7);
			//leg[0].setPosition(ANGLE,-1.57);
			//leg[2].setPosition(ANGLE,-1.57);
			//HIND LEGS
			leg[1].setOpenLoop(EXTENSION,1);
			leg[3].setOpenLoop(EXTENSION,1);
			leg[1].setPosition(ANGLE,.5);
			leg[3].setPosition(ANGLE,.5);

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
			leg[0].setPosition(EXTENSION,0.2);
			leg[2].setPosition(EXTENSION,0.2);
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
			leg[0].setPosition(EXTENSION,0.2);
			leg[2].setPosition(EXTENSION,0.2);
			leg[0].setPosition(ANGLE,.4);
			leg[2].setPosition(ANGLE,.4);
			// if(leg[0].getPosition(EXTENSION)<.6){
			// 	leg[0].setPosition(ANGLE,.4);
			// 	leg[2].setPosition(ANGLE,.4);
			// }

			if(leg[1].getPosition(ANGLE)<-.6){
				tEvent = millis();
				mode = RSUJ_AB;
				tarAng = -.7;
				tarAng2 = -.7;
				tarExt = .55;
				tarExt2 = .2;
			}
		break;


		 case RSUJ_AB :
		// 	tarAng = -.8;
		// 	leg[1].setPosition(EXTENSION,tarAng2);
		// 	leg[3].setPosition(EXTENSION,tarAng2);
		// 	leg[1].setPosition(ANGLE,tarAng);
			// leg[3].setPosition(ANGLE,tarAng);
			// if(tddet == true){
			// 	leg[0].setPosition(EXTENSION,0.6);
			// 	leg[2].setPosition(EXTENSION,0.6);
			// 	leg[0].setPosition(ANGLE,.3);
			// 	leg[2].setPosition(ANGLE,.3);
			// }else{
			// 	leg[0].setPosition(EXTENSION,0.8);
			// 	leg[2].setPosition(EXTENSION,0.8);
			// 	leg[0].setPosition(ANGLE,-.1);
			// 	leg[2].setPosition(ANGLE,-.1);

			// 	}
			// if(leg[0].getPosition(EXTENSION)<.55){
			// 	tddet = true;
			// }
		 	if(millis()-tEvent<75){
		 		//FRONT
		 		leg[0].setGain(ANGLE,.3);
				leg[2].setGain(ANGLE,.3);
		 		leg[0].setPosition(ANGLE,-.7); // was -1.2
				leg[2].setPosition(ANGLE,-.7);				
				//BACK
				leg[1].setPosition(EXTENSION,1.4);
				leg[3].setPosition(EXTENSION,1.4);

			}else if(millis()-tEvent>75 && millis()-tEvent<100){
				leg[0].setGain(ANGLE,.7);
				leg[2].setGain(ANGLE,.7);

				leg[0].setPosition(EXTENSION,.9);
				leg[2].setPosition(EXTENSION,.9);

				leg[0].setPosition(ANGLE,-.7);
				leg[2].setPosition(ANGLE,-.7);
			}else if(millis()-tEvent>100){
				tEvent = millis();
				mode = RSUJ_LANDED;
				tddetf = false;
				tddetr = false;
				tarAng = -.8;
			}// && millis()-tEvent<750){
				// if(leg[1].getPosition(EXTENSION)<1.1){
				// 	tddet=true;
				// }
				// if(tddet==false){

				// 	leg[1].setPosition(ANGLE,-.8);
				// 	leg[3].setPosition(ANGLE,-.8);
				// 	leg[0].setPosition(ANGLE,-1.2);
				// 	leg[2].setPosition(ANGLE,-1.2);
				// }else{
				// 	leg[1].setPosition(ANGLE,0);
				// 	leg[3].setPosition(ANGLE,0);
				// 	leg[0].setPosition(ANGLE,-.2);
				// 	leg[2].setPosition(ANGLE,-.2);
				// }
				// leg[1].setPosition(EXTENSION,1.2);
				// leg[3].setPosition(EXTENSION,1.2);
				// leg[0].setPosition(EXTENSION,1.2);
				// leg[2].setPosition(EXTENSION,1.2);

			// 	leg[1].setGain(EXTENSION,.6);
			// 	leg[3].setGain(EXTENSION,.6);
			// 	tarAng = -.4;
			// 	leg[1].setPosition(ANGLE,tarAng);
			// 	leg[3].setPosition(ANGLE,tarAng);
			// }else if(millis()-tEvent>300 && millis()-tEvent<500){
			// 	//leg[0].setPosition(EXTENSION,0.6);
			// 	//leg[2].setPosition(EXTENSION,0.6);
			// 	//leg[0].setPosition(ANGLE,0);
			// 	//leg[2].setPosition(ANGLE,0);
			// 	tarAng = -.3;
			// 	leg[1].setPosition(ANGLE,tarAng);
			// 	leg[3].setPosition(ANGLE,tarAng);
			// 	leg[0].setGain(ANGLE,.7);
			// 	leg[2].setGain(ANGLE,.7);
			// }else if(millis()-tEvent>500 && millis()-tEvent<1500){
			// 	tarAng = -.3;
			// 	leg[1].setPosition(ANGLE,tarAng);
			// 	leg[3].setPosition(ANGLE,tarAng);
			// 	leg[0].setGain(EXTENSION,.6);
			// 	leg[2].setGain(EXTENSION,.6);
			// }else if(millis()-tEvent>750){
			// 	mode = RSUJ_STAND;
			//	leg[0].setPosition(ANGLE,0);
			//	leg[1].setPosition(ANGLE,0);
			//	leg[2].setPosition(ANGLE,0);
			//	leg[3].setPosition(ANGLE,0);
				
			//	leg[0].setGain(ANGLE,1.3);
			//	leg[1].setGain(ANGLE,1.3);
			//	leg[2].setGain(ANGLE,1.3);
			//	leg[3].setGain(ANGLE,1.3);
				
				// leg[0].setGain(EXTENSION,1.5);
				// leg[1].setGain(EXTENSION,1.5);
				// leg[2].setGain(EXTENSION,1.5);
				// leg[3].setGain(EXTENSION,1.5);


				// leg[0].setPosition(EXTENSION,0.8);
				// leg[1].setPosition(EXTENSION,1.0);
				// leg[2].setPosition(EXTENSION,0.8);
				// leg[3].setPosition(EXTENSION,1.0);
				
			
		break;

		case RSUJ_LANDED :
			

			//Front LEG//////////////////
			// TD condition:
			if(leg[0].getPosition(EXTENSION)<.6 || leg[2].getPosition(EXTENSION)<.6){
				tddetf=true;
			}
			//PRIOR
			if(tddetf==false){
				leg[0].setPosition(EXTENSION,.9);
				leg[2].setPosition(EXTENSION,.9);

				leg[0].setPosition(ANGLE,-.7);
				leg[2].setPosition(ANGLE,-.7);
			//POST
			}else{
				leg[0].setPosition(EXTENSION,1.3);
				leg[2].setPosition(EXTENSION,1.3);
				leg[0].setPosition(ANGLE,-.2);
				leg[2].setPosition(ANGLE,-.2);
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
				leg[1].setPosition(ANGLE,-.8);
				leg[3].setPosition(ANGLE,-.8);
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
					mode = RSUJ_STAND;
				}
			}


		break;

	}
}
