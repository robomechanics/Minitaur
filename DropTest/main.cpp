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
int counter - 0;

float LP0[8] = { 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5 };
float LP1[8] = { 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5 };
float LP2[8] = { 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5 };
float LP3[8] = { 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5 };
float LP4[8] = { 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5 };
float LP5[8] = { 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5 };
float LP6[8] = { 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5 };
float LP7[8] = { 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5 };
float *last_pos[8] = {&LP0, &LP1, &LP2, &LP3, &LP4, &LP5, &LP6, &LP7};

float CP0[8] = { 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5 };
float CP1[8] = { 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5 };
float CP2[8] = { 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5 };
float CP3[8] = { 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5 };
float CP4[8] = { 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5 };
float CP5[8] = { 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5 };
float CP6[8] = { 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5 };
float CP7[8] = { 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5 };
float *cur_pos[8] = {&CP0, &CP1, &CP2, &CP3, &CP4, &CP5, &CP6, &CP7};

float hold_pos[8] = {0, 0, 0, 0, 0, 0, 0, 0};
bool crouched[8] = {false, false, false, false, false, false, false, false};

float avg(float &input_arr[8]) {
	float sum = 0;
	for (int i = 0; i < input_arr.size(); i++) {
		sum += input_arr[i];
	}
	return sum;
}

void update_pos(float &input_arr[8], float val) {
	for (int i = 1; i < input_arr.size(); i++) {
		input_arr[i] = input_arr[i - 1];
	}
	input_arr[0] = val;
}


enum FLMode {
	FL_WAIT_AIR = 0, FL_Falling, FL_Landing, FL_WAIT_GND
};
FLMode mode = FL_WAIT_AIR;
const float motZeros[9] = {2.570, 2.036, 3.777, 3.853, 2.183, 1.556, .675, 2.679, 2.61}; // RML Ellie w/aero tail

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
	
	void begin() {
		mode = FL_WAIT_AIR; //Start in wait mode
		prevTime = S->millis;// Set tLast at onset 
		C->mode = RobotCommand_Mode_JOINT;
		P->joints[8].type = JointParams_Type_GRBL;


// Set the *physical* address (e.g. etherCAT ID, PWM port, dynamixel ID, etc.)
		P->joints[8].zero = 2.61;
		P->joints[8].direction = 1;
		P->joints[8].address = 8;

		P->joints[8].gearRatio = 1.0;

		P->limbs_count = 0;
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
				joint[i].setGain(.03,.006);
			}
			joint[8].setGain(.2,.006);
			

			for (int i = 0; i < 8; ++i) {
				update_pos(cur_pos[i],joint[i].getPosition());
				if (!crouched[i]) {
					if (avg(cur_pos[i]) - avg(last_pos[i]) > 0.05) {
						hold_pos[i] = avg(last_pos[i]);
						joint[i].setPosition(hold_pos[i]);
						//joint[i].setGain(.2, .005);
						crouched[i] = true;
					}
					else {
						joint[i].setPosition(2.5);
					}
				}
				else {
					joint[i].setPosition(hold_pos[i]);
				}
				counter++;
				if (counter == 8) {
					*last_pos[i] = *cur_pos[i];
					counter = 0;
				}
			}
		}
		else if(mode == (FL_Falling)){
			fallTime = (S->millis - prevTime);

			
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
					//angDes = (isFront(i)) ? S->imu.euler.y - 0.1 : S->imu.euler.y + .1; // from first hop
				
					last_pos[i]  = joint[i].getPosition();
					joint[i].setGain(.8, .006);
					
					joint[i].setPosition(3.14+isOut(i)*angle-.5);
					
					
				}

			}
			if(fallTime > 50){//abs(S->imu.linear_acceleration.z) > 1){
				mode = FL_Landing;
				prevTime = S->millis;
						
			}
			
		}
		else if(mode == FL_Landing){

			joint[8].setPosition(3.14);
			
			

		}
		else if(mode == FL_WAIT_GND){


		}

	}

	bool running() {
		return true;
	}

	void end() {
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
	#define NUM_MOTORS 9
	//const float zeros[NUM_MOTORS] = {0, 0, 0, 0, 0, 0, 0, 0};
	const float directions[NUM_MOTORS] = {1, 1, 1, 1, -1, -1, -1, -1, 1};
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
	P->joints[8].address = 8;
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

