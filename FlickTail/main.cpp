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
float flickAngle;
bool newTest = true;
int startTime;



/**
 * State machine representation:
 * TS_WAIT -> stop the tail at 0 position
 * TS_FLICK -> spin the tail
 */
enum TSMode {
	TS_WAIT = 0, TS_FLICK
};
TSMode mode;
const float motZeros[10] = {2.570, 3.167, 3.777, 3.853, 2.183, 1.556, .675, 2.679, 1.000, 1.000}; // RML Ellie w/aero tail

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
	if(mode == TS_FLICK){
		float DF =joint[9].getOpenLoop();
		float curPos = joint[9].getPosition();
		int timeStamp = clockTimeUS - startTime;
		if(newTest){
			printf("XXXXXXXXXXX new trial, DF, Angle = ");
			printf("%f %f", DF, flickAngle);
			printf(" XXXXXXXXXXX\n");
			newTest = false;
		}
		printf("%f  %f  %i\n", theVel, curPos, timeStamp);
	}
}

class TailSpin : public Behavior {
public:
	//TSMode mode; //Current state within state-machine
	float DFarray[6] = {.1, .3, .5, .7, .9, 1}; // array of selectable duty factors
	float DF = .1;    //Desired duty factor to spin the tail, start at rest

	uint32_t tLast; // System time @ last velocity sample
	
	void begin() {
		mode = TS_WAIT; //Start in wait mode
	}

	//sig is mapped from remote; here, 3 corresponds to pushing the left stick to the right
	// which in turn forces the state machine into TS_FLICK
	void signal(uint32_t sig)
	{
		if(sig == 2) {
			flickAngle = 3.14159;
			mode = TS_FLICK;
			startTime = clockTimeUS;
		}
		if(sig == 3) {
			mode = TS_FLICK;
			flickAngle = 1.570795;
			startTime = clockTimeUS;
		}
		if(sig == 1 && abs(joint[9].getOpenLoop()) < .01 ) mode = TS_WAIT; // stop flicking
	}

	void update() {
		C->mode = RobotCommand_Mode_JOINT;
		if(mode == TS_WAIT){
			DF = DFarray[C->behavior.id];
			joint[9].setGain(0.1, 0.006); //Sets P and D gains for position based control
			joint[9].setPosition(0); // set the motor to 0 
			newTest = true;
		}
		else if(mode == TS_FLICK){
			if(joint[9].getPosition() < flickAngle && joint[9].getPosition() > -.3){
				joint[9].setOpenLoop(DF);
			}
			else{
				joint[9].setOpenLoop(0);
				mode = TS_WAIT;
			}
  			const int velocityBuffer = 5;
  			
  			static float driverVelocities [velocityBuffer];
  			static int counter = 0;
  			static bool filledUpArray = 0;
  			float betterDriverVelocity;
    		
    		driverVelocities[counter] = joint[9].getVelocity();

   			if(counter == velocityBuffer-1) filledUpArray = 1; //now we can average the Velocities
   			counter++;
    		if (filledUpArray == 1){
      			betterDriverVelocity = avg(driverVelocities, velocityBuffer);
      			theVel = betterDriverVelocity;
      			counter = 0;
      			filledUpArray = 0;
      		}      		
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
	#define NUM_MOTORS 10
	//const float zeros[NUM_MOTORS] = {0, 0, 0, 0, 0, 0, 0, 0};
	const float directions[NUM_MOTORS] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
	P->joints_count = S->joints_count = C->joints_count = NUM_MOTORS;
	for (int i = 0; i < P->joints_count; i++)
	{
		// Set zeros and directions
		P->joints[i].zero = motZeros[i];
		P->joints[i].direction = directions[i];
	}
	// Set the joint type; see JointParams
	P->joints[9].type = JointParams_Type_GRBL;

// Set the *physical* address (e.g. etherCAT ID, PWM port, dynamixel ID, etc.)
	P->joints[9].address = 8;

// If there is a gearbox the joint electronics doesn't know about, this could be > 1.
// Do not set to 0.
	P->joints[9].gearRatio = 1.0;
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
	setDebugRate(200);

	return begin();
}

