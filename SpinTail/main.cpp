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

float theVel;
float tailVel;
bool newTest = true;



/**
 * State machine representation:
 * TS_WAIT -> stop the tail at 0 position
 * TS_SPIN -> spin the tail
 */
enum TSMode {
	TS_WAIT = 0, TS_SPIN
};
TSMode mode;
const float motZeros[9] = {2.570, 3.167, 3.777, 3.853, 2.183, 1.556, .675, 2.679, 1.000}; // RML Ellie w/aero tail

float avg(float *myArray, int len){
  float sum = 0;
  float result;
  for (int i = 0; i < len; i++){
    sum += myArray[i];
  }
  result = sum/float(len);
  return result;
}

void debug(){
	if(mode == TS_SPIN){
		float DF =joint[8].getOpenLoop();
		if(newTest){
			printf("XXXXXXXXXXX new trial, DF= ");
			printf("%f", DF);
			printf(" XXXXXXXXXXX\n");
			newTest = false;
		}
		float theDF = joint[8].getOpenLoop();
		printf("%f\t %f\t %f\n", theVel, tailVel, theDF);
	}
}

class TailSpin : public Behavior {
public:
	//TSMode mode; //Current state within state-machine

	float posDes; //Desired position
	float curDF = .1;
	int dir = 1; // 1 or -1, direction to spin, + is for CCW - for CW
	float DFarray[6] = {.1, .3, .5, .7, .9, 1}; // array of selectable duty factors
	float DF = 0;    //Desired duty factor to spin the tail, start at rest

	uint32_t tLast; // System time @ last velocity sample
	
	void begin() {
		mode = TS_WAIT; //Start in wait mode
		posDes = 0; // Initialize desired position to zero
		tLast = S->millis;// Set tLast at onset 
	}

	//sig is mapped from remote; here, 3 corresponds to pushing the left stick to the right
	// which in turn forces the state machine into TS_SPIN
	void signal(uint32_t sig)
	{
		if(sig == 2) mode = TS_SPIN; // start the spinning
		if(sig == 3) mode = TS_WAIT; // stop the spinning
	}

	void update() {
		C->mode = RobotCommand_Mode_JOINT;
		curDF = joint[8].getOpenLoop();
		if(mode == TS_WAIT){
			//joint[8].setGain(0.01, 0.006); //Sets P and D gains for position based control
			//joint[8].setPosition(0); // set the motor to 0 
			//curDF = joint[8].getOpenLoop();
			if(S->millis - tLast > 80 && abs(curDF) > 0.01){
				curDF = dir*(abs(curDF)-.1);
				joint[8].setOpenLoop(curDF);
				tLast = S->millis;
			}
			if(abs(curDF) < .01){
				if(C->behavior.pose.position.z > .2) dir = 1;
				else if(C->behavior.pose.position.z < -.2) dir = -1;
				joint[8].setOpenLoop(0);
			}
			newTest = true;
		}
		else if(mode == TS_SPIN){
			if(S->millis - tLast > 100 && abs(curDF) - DFarray[C->behavior.id] > 0.05){
				curDF = dir*(abs(curDF)+.1);
				joint[8].setOpenLoop(curDF);
				tLast = S->millis;
			}
			else if(abs(curDF) - DFarray[C->behavior.id] < .01){
				curDF = DFarray[C->behavior.id]; // set duty factor using joystick knob
				joint[8].setOpenLoop(dir*curDF); //Set motor to spin at desired DF
			}
			static float prevPos = joint[8].getPosition();
 			static int prevTime = clockTimeUS;
  			int dt;
  			const int velocityBuffer = 500;
  			static float posVelocities [velocityBuffer];
  			static float driverVelocities [velocityBuffer];
  			static int counter = 0;
  			static bool filledUpArray = 0;
  			float pos;
  			float normalizedPos;
  			float dtheta;
  			float instantVelocity;
  			float betterPosVelocity;
  			float betterDriverVelocity;
  			//float tailVel;
  			
  			pos = joint[8].getPosition();
  			normalizedPos = pos;
  			if (pos < prevPos && dir == 1) {
    			normalizedPos = pos + 2*3.1415826;
  			}
  			else if (pos > prevPos && dir == -1){
  				normalizedPos = pos - 2*3.1415826;
  			}
  			//DF = joint[8].getOpenLoop();
  			
   			dtheta = normalizedPos - prevPos;
    		prevPos = pos;
    		dt = clockTimeUS-prevTime;
    		if(dt < 1){ 
    			dt = 1;
    		}

    		instantVelocity = 1000000.0 * dtheta/float(dt);
    		posVelocities[counter] = instantVelocity;
    		prevTime = clockTimeUS;
    		
    		driverVelocities[counter] = joint[8].getVelocity();
   			/*if (S->millis - tLast > 100){
   				posVelocities[counter] = joint[8].getVelocity();
   				counter++;
   			}*/

			/*if (curDF > .2) {
				curDF = .2;
			}
			else if (driverVelocities[counter] - 6.28 > .01) {
				curDF -= .0001;
			}
			else if (driverVelocities[counter] - 6.28 < -.01) {
				curDF += .0001;
			}
			joint[8].setOpenLoop(curDF*dir);*/

   			if(counter == velocityBuffer-1) filledUpArray = 1; //now we can average the posVelocities
   			counter++;
    		if (filledUpArray == 1){
      			betterPosVelocity = avg(posVelocities, velocityBuffer);
      			betterDriverVelocity = avg(driverVelocities, velocityBuffer);
      			tailVel = betterPosVelocity;
      			theVel = betterDriverVelocity;
      			counter = 0;
      			filledUpArray = 0;
      			//tLast = S->millis;
      		}
    		/*else{
      			betterVelocity = instantVelocity;
      			tailVel = betterVelocity;  
      		}*/
      		
      		
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
	/*for (int i = 0; i < P->joints_count+1; ++i)
		P->joints[i].zero = motZeros[i]; //Add motor zeros from array at beginning of file*/
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
	const float directions[NUM_MOTORS] = {1, 1, 1, 1, 1, 1, 1, 1, 1};
	P->joints_count = S->joints_count = C->joints_count = NUM_MOTORS;
	for (int i = 0; i < P->joints_count; i++)
	{
		// Set zeros and directions
		P->joints[i].zero = motZeros[i];
		P->joints[i].direction = directions[i];
	}
	// Set the joint type; see JointParams
	P->joints[8].type = JointParams_Type_GRBL;

// Set the *physical* address (e.g. etherCAT ID, PWM port, dynamixel ID, etc.)
	P->joints[8].address = 8;

// If there is a gearbox the joint electronics doesn't know about, this could be > 1.
// Do not set to 0.
	P->joints[8].gearRatio = 1.0;
	P->limbs_count = 0; 
	TailSpin tailSpin; //Declare instance of our behavior
	//Disable the safety shut off feature:
	//IT IS POSSIBLE TO DAMAGE THE MOTOR; BE CAREFUL WHEN USING
	//BEHAVIORS WITHOUT THIS FAILSAFE 
	safetyShutoffEnable(false);
	//Disable the softStart feature
	softStartEnable(false);
	//Remove default behaviors from behaviors vector
	behaviors.clear();
	//add our behavior to behaviors vector
	behaviors.push_back(&tailSpin);

	SerialPortConfig cfg;
	cfg.baud = 115200;
	cfg.mode = SERIAL_8N1;
	ioctl(STDOUT_FILENO, IOCTL_CMD_SERIAL_PORT_CFG, &cfg);
	setDebugRate(2);

	return begin();
}

