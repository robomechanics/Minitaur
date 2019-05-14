/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Gavin Kenneally, Avik De, Turner Topping <gavin@ghostrobotics.io> <avik@ghostrobotics.io> <turner@ghostrobotics.io>
 */
#include <stdio.h>
#include <SDK.h>
#include <math.h>
#include <Smath.h>
#include <Motor.h>
#include <Behavior.h>
#include <unistd.h>


float limb_forces[4] = {0,0,0,0};
float l1 = .1;
float l2 = .2;
float kt = 0.0954;
float R = .21;
unsigned long prevTime = S->millis;
float k = 0;
float c = 0;
float fallTime = 0;
float last_pos[8] = {3, 3, 3, 3, 3, 3, 3 ,3};
float cur_pos[8] = {0, 0, 0, 0, 0, 0, 0, 0};
float hold_pos[8] = {2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5};
bool crouched[8] = {false, false, false, false, false, false, false, false};


enum FLMode {
	FL_WAIT_AIR = 0, FL_Falling, FL_Landing, FL_WAIT_GND
};
FLMode mode = FL_WAIT_AIR;
const float motZeros[9] = {5.200, 5.712, 3.777, 3.853, 2.183, 1.556, .675, 2.679, 2.61}; // RML Ellie w/aero tail

bool isFront(int I){
	bool isFront;
	switch(I){
		case 0: isFront = true;
			break;
		case 1: isFront = true;
			break;
		case 2: isFront = false;
			break;
		case 3: isFront = false;
			break;	
		case 4: isFront = true;
			break;
		case 5: isFront = true;
			break;
		case 6: isFront = false;
			break;
		case 7: isFront = false;
			break;		
	}		
	return isFront;
}

float isOut(int I){
	// determine of motor is outboard or inboard
	float isOut;
	switch(I){
		case 0: isOut = 1;
			break;
		case 1: isOut = -1;
			break;
		case 2: isOut = 1;
			break;
		case 3: isOut = -1;
			break;	
		case 4: isOut = -1;
			break;
		case 5: isOut = 1;
			break;
		case 6: isOut = -1;
			break;
		case 7: isOut = 1;
			break;		
	}		
	return isOut;
}

void debug(){
	printf("%f\t%f\n", S->imu.linear_acceleration.x, joint[8].getPosition());
}

class TailSpin : public Behavior {
public:


	uint32_t tLast; // System time @ last velocity sample

	void signal(uint32_t sig) {
		if (sig == 3) {
		}
	}
	
	void begin() {



		mode = FL_WAIT_AIR;//Start in wait mode
		prevTime = S->millis;// Set tLast at onset 
		C->mode = RobotCommand_Mode_JOINT;

		

// Set the *physical* address (e.g. etherCAT ID, PWM port, dynamixel ID, etc.)
		

		
		for(int i = 0; i < 8; i++){
			joint[i].setGain(.8,.006);
			joint[i].setPosition(2.5);
		}
	}

	//sig is mapped from remote; here, 3 corresponds to pushing the left stick to the right
	// which in turn forces the state machine into TS_SPIN
	/*void signal(uint32_t sig)
	{

	}*/

	void update() {

		/*if(C->behavior.mode == 0){
			return;
		}
		if(C->behavior.id != 0){
			return;
		}*/
		if(mode == FL_WAIT_AIR){
		//float extension = map(C->behavior.pose.position.z, -1.0, 1.0, 0.2, 0.3);

		// And angle is calculated as the negative pitch value of the robot to keep the legs pointing down.
		//float angle = -S->imu.euler.y;
		
		
			for(int i = 0; i < 8; i++){
				// hold legs in start position
				joint[i].setGain(.8,.006);
				joint[i].setPosition(2.5);
			}
			joint[8].setGain(.2,.006);
			joint[8].setPosition(0);

			if(abs(S->imu.linear_acceleration.x) < 4.5 && S->millis - prevTime > 5000){
				// when released switch to falling
			//if(S->millis - prevTime > 5000){//} && abs(S->imu.linear_acceleration.z < 6)){
				mode = FL_Falling;
				prevTime = S->millis;
			}
		}
		else if(mode == (FL_Falling)){
			fallTime = (S->millis - prevTime);

			// flick the tail if it hasn't reahed end of travel
			if(joint[8].getPosition() < 2.9 && joint[8].getPosition() > -1){
				joint[8].setOpenLoop(-1);
			}	
			else{
				joint[8].setPosition(3.14159);
			}
			
			
			float angle = S->imu.euler.y;
			
			if(abs(angle < 1.2)){
				for (int i = 0; i < 8; ++i)
				{
					joint[i].setPosition(3.14+isOut(i)*angle-.5);
				}

			}
			if(abs(angle) < .3 || S->millis-prevTime > 5000){//abs(S->imu.linear_acceleration.z) > 1){
				// when legs pointed down start preparing to land
				mode = FL_Landing;
				prevTime = S->millis;		
			}
			
		}
		else if(mode == FL_Landing){

			joint[8].setPosition(0);
			
			for (int i = 0; i < 8; ++i){
				// legs "ratchet" such that they can resist collaps but not extend
				cur_pos[i] = joint[i].getPosition();
				if(cur_pos[i] < hold_pos[i]){
					hold_pos[i] -= .015;//cur_pos[i];
					joint[i].setPosition(hold_pos[i]);
					//joint[i].setGain(.2,.005);
					crouched[i] = true;

				}
				else{
					joint[i].setPosition(hold_pos[i]);
				}
				last_pos[i] = cur_pos[i];
			}	

		}
		else if(mode == FL_WAIT_GND){


		}

	}

	bool running() {
		return true;
	}

	void end() {
		P->limbs_count = 4; 
	}
};

int main(int argc, char *argv[]) {

	#if defined(ROBOT_MINITAUR)
	init(RobotParams_Type_MINITAUR, argc, argv);

		
	#elif defined(ROBOT_MINITAUR_E)
	init(RobotParams_Type_MINITAUR_E, argc, argv);
	JoyType joyType = JoyType_FRSKY_XSR;
	ioctl(JOYSTICK_FILENO, IOCTL_CMD_JOYSTICK_SET_TYPE, &joyType);
	#else
	#error "Define robot type in preprocessor"
	#endif

	// Configure joints
	#define NUM_MOTORS  9
	//const float zeros[NUM_MOTORS] = {0, 0, 0, 0, 0, 0, 0, 0};
	float directions[NUM_MOTORS] = {1, 1, 1, 1, -1, -1, -1, -1, 1};
	P->joints_count = S->joints_count = C->joints_count = NUM_MOTORS;
	for (int i = 0; i < P->joints_count; i++)
	{
		// Set zeros and directions
		P->joints[i].zero = motZeros[i];
		P->joints[i].direction = directions[i];
	}
	// Set the joint type; see JointParams
	P->joints[8].type = JointParams_Type_GRBL;
	//P->joints[9].type = JointParams_Type_GRBL;

// Set the *physical* address (e.g. etherCAT ID, PWM port, dynamixel ID, etc.)
	P->joints[8].address = 9;
	//P->joints[9].address = 9;

// If there is a gearbox the joint electronics doesn't know about, this could be > 1.
// Do not set to 0.
	P->joints[8].gearRatio = 1.0;
	//P->joints[9].gearRatio = 1.0;
	P->limbs_count = 0; 
	/*limb[4].setDims(2,2);
	P->limbs[4].jointInd_count = 2;
	P->limbs[4].jointInd[0] = 8; // j0 is the index into joint[]
	P->limbs[4].jointInd[1] = 9;
	P->limbs[4].type = LimbParams_Type_SYMM5BAR_EXT_M;*/
	TailSpin tailSpin; //Declare instance of our behavior
	//Disable the safety shut off feature:
	//IT IS POSSIBLE TO DAMAGE THE MOTOR; BE CAREFUL WHEN USING
	//BEHAVIORS WITHOUT THIS FAILSAFE 
	safetyShutoffEnable(true);
	//Disable the softStart feature
	softStartEnable(true);
	//Remove default behaviors from behaviors vector
	
	//add our behavior to behaviors vector
	behaviors.push_back(&tailSpin);
	

	return begin();
	
}

