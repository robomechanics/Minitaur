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
#include <ReorientableBehavior.h>
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
int touchdowns = 0;

enum FLMode {
	FL_WAIT_AIR = 0, FL_Falling, FL_Landing, FL_WAIT_GND
};
FLMode mode;
const float motZeros[9] = {2.570, 2.036, 3.777, 3.853, 2.183, 1.556, .675, 2.679, 2.61}; // RML Ellie w/aero tail

bool isFront(int I){
	bool isfront = I % 2;
	return isfront;
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
	}

	//sig is mapped from remote; here, 3 corresponds to pushing the left stick to the right
	// which in turn forces the state machine into TS_SPIN
	/*void signal(uint32_t sig)
	{

	}*/

		void update() {
		if(mode == FL_WAIT_AIR){
		//float extension = map(C->behavior.pose.position.z, -1.0, 1.0, 0.2, 0.3);

		// And angle is calculated as the negative pitch value of the robot to keep the legs pointing down.
		//float angle = -S->imu.euler.y;

		
			for(int i = 0; i < 8; i++){
				joint[i].setGain(.1,.006);
				joint[i].setPosition(1.57);
			}
			joint[8].setGain(.2,.006);
			joint[8].setPosition(0);

			if(abs(S->imu.linear_acceleration.x) < 4.5 && S->millis - prevTime > 5000){
				mode = FL_Falling;
				prevTime = S->millis;
			}
		}
		else if(mode == (FL_Falling)){
			fallTime = (S->millis - prevTime);
			//float angDes;
			//float angle = S->imu.euler.y;

			
			if(joint[8].getPosition() < 2.5 && joint[8].getPosition() > -1){
				joint[8].setOpenLoop(-1);
			}	
			else{
				joint[8].setPosition(3.14159);
			}
			
			for(int i = 0; i < 8; i++){
				joint[i].setPosition(1.57);
			}
			
			/*if(abs(angle < .8)){
				for (int i = 0; i < P->limbs_count; ++i)
				{
					angDes = (isFront(i+1)) ? -S->imu.euler.y - 0.1 : -S->imu.euler.y + 0.2; // from first hop
				// Set leg angles
					limb[i].setGain(ANGLE, 0.8, 0.03);
					limb[i].setGain(EXTENSION, 60, 3);
					limb[i].setPosition(ANGLE, angDes);
					limb[i].setPosition(EXTENSION, .24);
					if(limb[i].getForce(EXTENSION) > 20 && fallTime > 100){
						mode = FL_Landing;
						prevTime = S->millis;
					}
				}

			}*/
		}
		else if(mode == FL_Landing){

			C->mode = RobotCommand_Mode_JOINT;
			joint[8].setOpenLoop(0);
			C->mode = RobotCommand_Mode_LIMB;


			for(int i = 0; i < 4; i++){
				limb[i].setPosition(EXTENSION, .24);
				//P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_RAD;
				//limb[i].setOpenLoop(EXTENSION,1);
				/*if(limb[i].getVelocity(EXTENSION) > 0 && S->millis - prevTime > 25){
					mode = FL_WAIT_GND;
					prevTime = S->millis;
				}*/
			}
		}
		/*else if(mode == FL_WAIT_GND){
			float extension = map(C->behavior.pose.position.z, -1.0, 1.0, 0.15, 0.16);

			C->mode = RobotCommand_Mode_JOINT;
			joint[8].setOpenLoop(0);
			C->mode = RobotCommand_Mode_LIMB;
		// And angle is calculated as the negative pitch value of the robot to keep the legs pointing down.
			float angle = -S->imu.euler.y;

		// For each of the four legs:
			for (int i = 0; i < P->limbs_count; ++i)
			{
			// Set limb type
				P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_M;

			// Set leg angles
				limb[i].setGain(ANGLE, 0.8, 0.03);
				limb[i].setPosition(ANGLE, angle);

			// Set leg extensions
				limb[i].setGain(EXTENSION, 70, 3);
				limb[i].setPosition(EXTENSION, extension);
			}
			/*if(S->millis-prevTime > 10000){
				mode = FL_WAIT_AIR;
			}
		}*/
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
	behaviors.clear();
	//add our behavior to behaviors vector
	behaviors.push_back(&tailSpin);

	SerialPortConfig cfg;
	cfg.baud = 115200;
	cfg.mode = SERIAL_8N1;
	ioctl(STDOUT_FILENO, IOCTL_CMD_SERIAL_PORT_CFG, &cfg);
	setDebugRate(100);

	return begin();
}

