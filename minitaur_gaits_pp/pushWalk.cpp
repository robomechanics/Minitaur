/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#include "pushWalk.h"
// leaps as part of walk ("sub-behavior"?)
#include "RSUJump.h"

/**
 * TODO:
 * input filtering to avoid falls
 * see if speed can increase
 * two leaps integrated
 * see if speed can increase
 * try odometry compare to ground truth
 * legs get caught in rocks: check leg extension in flight phase, if did not get small enough, leg must have gotten caught...do something??
 * energy efficiency: look at logs to see forces (where is energy going?)
 * touchdown detection
 * start from remote signal: step over, let legs out of sync to tripod
 */

pushWalk pushwalk;

void pushWalk::signal(uint8_t sig) {
  // FIXME for now use this for step over, later replace with proprio
  // use LEAP_STANCE to make update do the jump update instead
  signalState = SIGNAL_LEAP_STANCE;
  rsuJump.begin();
}

void pushWalk::update() {
  // Leap sub-behaviors -----------------------
  if (signalState == SIGNAL_LEAP_STANCE) {
    rsuJump.update();
    if (!rsuJump.running())
      signalState = SIGNAL_NONE;
    return;
  }
  // Walk code ------------------------------
  MinitaurLeg::useLengths = false;
  X.mode = mode;// log walkmode

  // For inverted operation and roll/pitch recovery ---------------------------
  // 1) take it out of walking mode if not close to nominal
  if (isReorienting()) {
    this->mode = pWM_REORIENT;
    return;
  }
  // for the rest of the code, either bInverted or bUpright is true; use either to check
  // if running in inverted mode.

  // SIT MODE ------------------------------
  if (mode == pWM_SIT/* || mode == pWM_WAIT*/) {
    // stand
    for (int i=0; i<4; ++i) {
      leg[i].setGain(EXTENSION, 0.2);
      leg[i].setGain(ANGLE, 0.4, 0.01);
      float angDes = (i==0||i==2) ? 0.05 : 0.15;
      if (bInverted)
        angDes = (i==0||i==2) ? 0.15 : 0.05;
      // Height using remote
      leg[i].setPosition(EXTENSION, map(vertDes, 0.0, 1.0, 0.4, 0.8));
      leg[i].setPosition(ANGLE, angDes - X.pitch);
    }
    // hitting remote switch moves to wait, and then it waits here 200ms before enabling
    // if (mode == pWM_WAIT && X.t - tstart > 200)
    //   mode = pWM_WALK;
    X.xd = 0;//presumably
    return;
  }
  // Assume always running (or other behaviors can switch into it)
  if (X.t - lastUpdate > 100) {
    init();
  }
  lastUpdate = X.t;

  // SOME PARAMETERS HERE, SOME BELOW -------------------
  // const float kSpeed = 0.2;//how the COM reacts to remote
  // flightLeg == -1 allows some leg to lift off
  // if flightLeg >= 0, all other legs are inhibited from lifting off

  // Times
  const int tflight = 170, tminstance = 100;
  // Gains attitude control
  const float kPitchD = 0.05;
  // const float kRoll = 1.5, kRollD = 0.04;
  const float kRoll = 4.0, kRollD = 0.02;
  // Gains position control
  // const float kExtPStance = 0.3, kExtDStance = 0.02;
  const float kExtPStance = 0.5, kExtDStance = 0.01;
  const float kExtPFlight = 0.6;//0.4 for lighter legs
  const float kAngPFlight = 0.4;// 0.2 for lighter legs
  // Positions
  // float extDes = 2.5;//stance extension
  // TEST height using remote
  float extDes = map(vertDes, 0, 1, 1.0, 2.5);
  const float extMin = 0.7;//0.5 for light legs minimum extension in retraction
  const float kPEPthresh = 0.15;//0.1;// (i==1 || i==3) ? 0.3 : 0.1; //0.3 and 0.1
  // Forces
  // const float kTDthresh = 5;//;


  // Variables ----------------------------------------------------------------
  float rollDes = 0;//(nextFlightLeg > 1) ? -0.1 : 0.1;
  // float rollDes = yawDes;
  float uroll = kRoll*(X.roll - rollDes) + kRollD*X.rolldot;
  float uspeed = 0.003*speedDes;
  // body pitch should conform to slope (add pitch damping), but roll should correct to 0. instead legs should point vertically down
  float upitch = kPitchD*X.pitchdot;

  // Trot walk ----------------------------------------------
  // flightLeg will be -1, 0, or 1
  float speedAccum = 0;
  int numInStance = 0;
  //For logging
  X.mode = 10+flightLeg;
  for (int i=0; i<3; i+=2) {
    leg[i].setGain(EXTENSION, 0.5);
    leg[i].setGain(ANGLE, 0.5);
    leg[i].setPosition(EXTENSION, 0.2);
    leg[i].setPosition(ANGLE, -1.8);
  }
  for (int i=1; i<4; i+=2) {                // CHANGED THIS TO LOOP FOR LEGS 1 and 3
    bool bRear = (i==1 || i==3);
    if (bInverted)
      bRear = !bRear;
    bool bRight = (i>1);
    // IMPORTANT: set nominal leg angles
    float angNom = bRear ? 0.3 : 0.0;      // Changed this to angle back legs further back
    // get positions
    float ext = leg[i].getPosition(EXTENSION);
    float extvel = leg[i].getVelocity(EXTENSION);

    // Diagonal pair
    // Normally two legs in flight, but if stepping is activated only stepLeg is in flight
    if ((stepLeg == -1 && (flightLeg == i || flightLeg + i == 3)) || (stepLeg == i)) {
      // pair in flight
      // FLIGHT CONTROL
      frac = map(X.t, tLO, tLO + tflight, 0.0, 1.0);
      // step over obstacle (IGNORE FOR NOW)
      // FIXME use remote for now
      if (signalState == SIGNAL_QUEUE && frac > 0.5) {
        // proprio will set a stepLeg based on which leg to step with
        // for now flightLeg is 0 or 1, pick the front leg in each of those sets
        if (nextStepLeg == -1 && stepLeg == -1)
          nextStepLeg = (flightLeg == 0) ? 0 : 2;
        if (nextStepLeg == i)
          frac = 0.55;// stop
      }

      // float extGain = (frac < 0.5) ? 0.5 : 0.1;
      leg[i].setGain(EXTENSION, (frac < 0.5) ? kExtPFlight : 0.05);
      leg[i].setGain(ANGLE, (frac < 0.5) ? kAngPFlight : 0.1);
      // retract for flight path
      float extRetract = (extDes - extMin);
      // try to soften touchdown FIXME touchdown detection
      if (frac < 0.6)
        leg[i].setPosition(EXTENSION, extDes - extRetract * arm_sin_f32(frac * PI));
      else
        leg[i].setOpenLoop(EXTENSION, 0.075);// 0.05 on lighter legs
      // AEP based on PEP
      const float aep = -1.0*pep;//max(-1*pep, -1);
      // if this is the other one within the pair when a nextStepLeg is selected
      if (nextStepLeg+i == 3) {
        absAngles[i] += 0.005 * (-0.1 - absAngles[i]);
      } else {
        absAngles[i] = (1-frac)*pep + frac*aep;
      }
      leg[i].setPosition(ANGLE, angNom + absAngles[i] - X.pitch);
      // TOUCHDOWN
      // leg inertial forces are too high for touchdown detection
      if (/*ur[i] > kTDthresh && */frac >= 0.8) {
        if (nextStepLeg != -1) 
          stepLeg = nextStepLeg;
        else {
          flightLeg = -1;
          // nextStepLeg, stepLeg will both be reset to -1 after the step
          nextFlightLeg = (i==0 || i==3) ? 1 : 0;// i == 0/3 -> 1, i == 1/2->0
          tTD = X.t;
        }
      }
    } else {
      bool legJustTouchedDown = !(nextFlightLeg == i || nextFlightLeg + i == 3);
      // accumulate speed for stance legs
      // for legs that just touched down wait 50ms after TD
      if (X.t - tTD > 50 || !legJustTouchedDown) {
        // there is some lag between the flightLeg being set and the 
        // leg actually touching down (from looking at logs)
        speedAccum += leg[i].getSpeed(X.pitch);
        numInStance++;
      }
      // Leg i is in stance. STANCE CONTROL - try to keep body level
      leg[i].setGain(ANGLE, 1.0, 0.015);
      // lean forward based on speedDes, but add some decay
      float uyaw = bRight ? -0.05*yawDes : 0.05*yawDes;
      if (flightLeg == -1 || fabsf(absAngles[i] > 0.3)) uyaw = 0;
      if (X.t - tTD > 25 || !legJustTouchedDown)
        absAngles[i] += uspeed + uyaw - 0.001*absAngles[i];
      // TEST
      if (nextStepLeg != -1) {
        // move back
        absAngles[i] += 0.005 * (-0.1 - absAngles[i]);
        // leg behind or across
        // else
        //   extDes = 2.8;
      }

      leg[i].setPosition(ANGLE, angNom + absAngles[i] - X.pitch);
      // apply to legs correctly
      float rollCtrl = bRight ? uroll : -uroll;
      float pitchCtrl = bRear ? -upitch : upitch;

      if (nextStepLeg != -1) {
        // try to get leg to not EXTEND for roll control
        if (rollCtrl > 0)
          rollCtrl = 0;
        // pitchCtrl = 0;
      }

      leg[i].setOpenLoop(EXTENSION, kExtPStance*(extDes - ext) - kExtDStance*extvel + rollCtrl + pitchCtrl);
    }
    // liftoff 
    if (flightLeg==-1 && X.t - tTD > tminstance) {
      // based on yawDes or PEP
      if (fabsf(yawDes) > 0.05 || fabsf(absAngles[i]) > kPEPthresh) {
        flightLeg = nextFlightLeg;
        tLO = X.t;
        pep = absAngles[i];
      }
    }
  }
  // speed (filter?)
  if (numInStance > 0)
    X.xd = speedFilt.update(speedAccum /((float)numInStance));
}


// // CRAWL (NOT USED NOW) -------------------------------------------------
// const float kLOthreshRear = 20, kLOthreshFront = 35;

// for (int i=0; i<4; ++i) {
//   bool bRear = (i==1 || i==3);
//   bool bRight = (i>1);
//   // get toe radial force
//   float uth;
//   leg[i].getToeForce(ur[i], uth);
//   if (bRight) ur[i] = -ur[i];
//   // get positions
//   float ext = leg[i].getPosition(EXTENSION);
//   float extvel = leg[i].getVelocity(EXTENSION);

//   float angNom = bRear ? 0.05 : -0.05;
//   // float angDes = angNom;

//   // NEED TO LIFT OFF DIAGONAL LEG
//   float uyaw = bRight ? -0.01*yawDes : 0.01*yawDes;
//     // // TEST
//     // if (fabsf(absAngles[i]) > 0.6)
//     //   uyaw = 0;

//   // All legs
//   if (flightLeg != i) {
//     // STANCE CONTROL - try to keep body level
//     leg[i].setGain(ANGLE, 1.0, 0.015);
//     // lean forward based on speedDes, but add some decay
//     absAngles[i] += uspeed + uyaw - 0.001*absAngles[i];
//     leg[i].setPosition(ANGLE, angNom + absAngles[i] - constrain(X.pitch, -1, 1));
//     // apply to legs correctly
//     float rollCtrl = bRight ? uroll : -uroll;
//     float pitchCtrl = bRear ? -upitch : upitch;
//     // TEST for yaw lift off diagonally opposite leg
//     if (flightLeg + i == 3 && fabsf(yawDes) > 0.05) {
//       // i is diagonally across
//       leg[i].setOpenLoop(EXTENSION, kExtPStance*(extDes - 1.0 - ext));
//     } else {
//       // use leg extensions to correct for orientation
//       leg[i].setOpenLoop(EXTENSION, kExtPStance*(extDes - ext) - kExtDStance*extvel + rollCtrl + pitchCtrl);
//     }
//     // if (abs(nextFlightLeg - i) == 2 || abs(flightLeg - i) == 2)
//     //   extDes += 0.2;
//   } else if (flightLeg == i) {
//     // FLIGHT CONTROL
//     float frac = constrain(interpFrac(tLO, tLO + tflight, X.t), 0, 1);
//     // float extGain = (frac < 0.5) ? 0.5 : 0.1;
//     leg[i].setGain(EXTENSION, (frac < 0.5) ? kExtPFlight : 0.1);
//     leg[i].setGain(ANGLE, (frac < 0.5) ? kAngPFlight : 0.1);
//     // retract for flight path
//     float extRetract = (extDes - extMin);
//     leg[i].setPosition(EXTENSION, extDes - extRetract * arm_sin_f32(frac * PI)); //nominally 1.5
//     // AEP based on PEP
//     const float aep = max(-3 * pep, -1);
//     absAngles[i] =  (1-frac)*pep + frac*aep;
//     leg[i].setPosition(ANGLE, angNom + absAngles[i] - constrain(X.pitch, -1, 1));
//     // TOUCHDOWN based on force
//     if (ur[i] > kTDthresh && frac >= 0.4) {
//       flightLeg = -1;
//       nextFlightLeg = (i + 3) % 4;// decrement i
//       tTD = X.t;
//     }
//   }
//   // LIFTOFF
//   float LOthresh = bRear ? kLOthreshRear : kLOthreshFront;
//   // sequenced hybrid
//   if (flightLeg == -1 && nextFlightLeg == i) {
//     if (ur[i] < LOthresh && absAngles[i] > kPEPthresh) {
//       flightLeg = nextFlightLeg;
//       tLO = X.t;
//       pep = absAngles[i];
//     }
//   }
// }

// return;

