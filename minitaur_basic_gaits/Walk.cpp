
#include "Walk.h"

Walk walk;


void Walk::update() {
  MinitaurLeg::useLengths = false;
  // SOME PARAMETERS HERE, SOME BELOW -------------------
  // const float kSpeed = 0.2;//how the COM reacts to remote
  // flightLeg == -1 allows some leg to lift off
  // if flightLeg >= 0, all other legs are inhibited from lifting off

  // Times
  const int tflight = 150, tminstance = 100;
  // Gains attitude control
  const float kPitchD = 0.05;
  // const float kRoll = 1.5, kRollD = 0.04;
  const float kRoll = 3.0, kRollD = 0.03;
  // Gains position control
  // const float kExtPStance = 0.3, kExtDStance = 0.02;
  const float kExtPStance = 0.5, kExtDStance = 0.01;
  const float kExtPFlight = 0.4;
  const float kAngPFlight = 0.2;
  // Positions
  // float extDes = 2.5;//stance extension
  // TEST height using remote
  float extDes = map(vertDes, 0, 1, 1.0, 2.5);
  const float extMin = 0.5;//minimum extension in retraction
  const float kPEPthresh = 0.15;//0.1;// (i==1 || i==3) ? 0.3 : 0.1; //0.3 and 0.1
  // Forces
  // const float kTDthresh = 5;//;

  // Variables ---------------------------
  float rollDes = 0;//(nextFlightLeg > 1) ? -0.1 : 0.1;
  // float rollDes = yawDes;
  float uroll = kRoll*(X.roll - rollDes) + kRollD*rolldot;
  float uspeed = 0.005*speedDes;
  // body pitch should conform to slope (add pitch damping), but roll should correct to 0. instead legs should point vertically down
  float upitch = kPitchD*pitchdot;


  // SIT MODE ------------------------------
  if (mode == WM_SIT || mode == WM_WAIT) {
    // stand
    for (int i=0; i<4; ++i) {
      leg[i].setGain(EXTENSION, 0.2);
      leg[i].setGain(ANGLE, 0.4);
      float angDes = (i==0||i==2) ? 0.05 : 0.15;
      leg[i].setPosition(ANGLE, angDes - constrain(X.pitch, -1, 1));
      // leg[i].setPosition(EXTENSION, 0.8);
  // TEST height using remote
      leg[i].setPosition(EXTENSION, map(vertDes, 0, 1, 0.4, 0.8));
    }
    // hitting remote switch moves to wait, and then it waits here 200ms before enabling
    if (mode == WM_WAIT && X.t - tstart > 200)
      mode = WM_WALK;
    return;
  }
  // Assume always running
  if (X.t - lastUpdate > 100) {
    init();
  }
  lastUpdate = X.t;

  // TROT TEST YAW ONLY ----------------------------------------------
  // flightLeg will be -1, 0, or 1
  for (int i=0; i<4; ++i) {
    bool bRear = (i==1 || i==3);
    bool bRight = (i>1);
    float angNom = bRear ? 0.1 : 0.0;
    // get positions
    float ext = leg[i].getPosition(EXTENSION);
    float extvel = leg[i].getVelocity(EXTENSION);

    // Diagonal pair
    // Normally two legs in flight, but if stepping is activated only stepLeg is in flight
    if ((stepLeg == -1 && (flightLeg == i || flightLeg + i == 3)) || (stepLeg == i)) {
      // pair in flight
      // FLIGHT CONTROL
      frac = constrain(interpFrac(tLO, tLO + tflight, X.t), 0, 1);


      // step over obstacle
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
        leg[i].setOpenLoop(EXTENSION, 0.05);
      // AEP based on PEP
      const float aep = max(-1*pep, -1);
      // if this is the other one within the pair when a nextStepLeg is selected
      if (nextStepLeg+i == 3) {
        absAngles[i] += 0.005 * (-0.1 - absAngles[i]);
      } else {
        absAngles[i] =  (1-frac)*pep + frac*aep;
      }
      leg[i].setPosition(ANGLE, angNom + absAngles[i] - constrain(X.pitch, -1, 1));

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
      // Leg i is in stance. STANCE CONTROL - try to keep body level
      leg[i].setGain(ANGLE, 1.0, 0.015);
      // lean forward based on speedDes, but add some decay
      float uyaw = bRight ? -0.05*yawDes : 0.05*yawDes;
      if (flightLeg == -1 || fabsf(absAngles[i] > 0.3)) uyaw = 0;
      absAngles[i] += uspeed + uyaw - 0.001*absAngles[i];

      // TEST
      if (nextStepLeg != -1) {
        // move back
        absAngles[i] += 0.005 * (-0.1 - absAngles[i]);
        // leg behind or across
        // else
        //   extDes = 2.8;
      }

      leg[i].setPosition(ANGLE, angNom + absAngles[i] - constrain(X.pitch, -1, 1));
      // apply to legs correctly
      float rollCtrl = bRight ? uroll : -uroll;
      float pitchCtrl = bRear ? -upitch : upitch;

      // TEST
      // if (nextStepLeg != -1 && i+nextStepLeg==2)
      //   leg[i].setOpenLoop(EXTENSION, 0.3);
      // else {

        if (nextStepLeg != -1) {
          // try to get leg to not EXTEND for roll control
          if (rollCtrl > 0)
            rollCtrl = 0;
          // pitchCtrl = 0;
        }
          // if ((nextStepLeg==0 && i==2) || (nextStepLeg==2 && i==0)) {
          //   extDes = 2.8;
          // } else {
            // extDes = 0.7*extDes;
          // }
        // } else {
          leg[i].setOpenLoop(EXTENSION, kExtPStance*(extDes - ext) - kExtDStance*extvel + rollCtrl + pitchCtrl);
      // }
        // }
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
  return;

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
}

