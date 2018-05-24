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



/**
 * State machine representation:
 * TS_WAIT -> stop the tail at 0 position
 * TS_SPIN -> spin the tail
 */
enum TSMode {
	TS_WAIT = 0, TS_SPIN
};
TSMode mode;
const float motZeros[8] = {2.570, 3.167, 3.777, 3.853, 2.183, 1.556, .675, 2.679}; // RML Ellie

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
		printf("%f\n", theVel);
	}
}

class TailSpin : public Behavior {
public:
	//TSMode mode; //Current state within state-machine

	float posDes; //Desired position
	float curDF;
	int dir = 1; // 1 or -1, direction to spin, + is for CCW - for CW
	float DFarray[6] = {.1, .3, .5, .7, .9, 1}; // array of selectable duty factors
	float DF = dir*DFarray[C->behavior.id];    //Desired duty factor to spin the tail

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
		if(mode == TS_WAIT){
			//joint[0].setGain(0.01, 0.006); //Sets P and D gains for position based control
			//joint[0].setPosition(0); // set the motor to 0 
			curDF = joint[0].getOpenLoop();
			if(S->millis - tLast > 50 && curDF > 0.05){
				curDF -= .1*dir;
				joint[0].setOpenLoop(curDF);
				tLast = S->millis;
			}
			if(C->behavior.pose.position.z > .2) dir = 1;
			else if(C->behavior.pose.position.z < -.2) dir = -1;
		}
		else if(mode == TS_SPIN){
			DF = DFarray[C->behavior.id]; // set duty factor using joystick knob
			joint[0].setOpenLoop(DF);//Set motor to spin at desired DF
			/*static float prevPos = joint[0].getPosition();
  			static int initialize = 1;
 			//static int prevTime = S->millis;
  			int dt;*/
  			const int velocityBuffer = 100;
  			static float velocities [velocityBuffer];
  			static int counter = 0;
  			static bool filledUpArray = 0;
  			/*float pos;
  			float normalizedPos;
  			float DF;
  			float dtheta;
  			float instantVelocity;*/
  			float betterVelocity;
  
  			/*pos = joint[0].getPosition();
  			normalizedPos = pos;
  			if (pos < prevPos) {
    			normalizedPos = pos + 2*3.1415826;
  			}
  			DF = joint[0].getOpenLoop();
  			theVel = joint[0].getVelocity();
   			dtheta = normalizedPos - prevPos;
    		if (dtheta < 0){
    			theVel = pos;
    		}
    		dt = S->millis-prevTime;
    		instantVelocity = 1000.0 * dtheta/float(dt);
    		velocities[counter] = instantVelocity;*/
    		
    		
   			if (S->millis - tLast > 100){
   				velocities[counter] = joint[0].getVelocity();
   				counter++;
   			}

   			if(counter == velocityBuffer-1) filledUpArray = 1; //now we can average the velocities
   			
    		if (filledUpArray == 1){
      			betterVelocity = avg(velocities, counter);
      			theVel = betterVelocity;
      			counter = 0;
      			filledUpArray = 0;
      			tLast = S->millis;
      		}
    		/*else{
      			betterVelocity = instantVelocity;
      			theVel = betterVelocity;  
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
	for (int i = 0; i < P->joints_count; ++i)
		P->joints[i].zero = motZeros[i]; //Add motor zeros from array at beginning of file
	#elif defined(ROBOT_MINITAUR_E)
	init(RobotParams_Type_MINITAUR_E, argc, argv);
	JoyType joyType = JoyType_FRSKY_XSR;
	ioctl(JOYSTICK_FILENO, IOCTL_CMD_JOYSTICK_SET_TYPE, &joyType);
	#else
	#error "Define robot type in preprocessor"
	#endif
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
	setDebugRate(10);

	return begin();
}

