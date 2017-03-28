/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#include "Bound.h"

Bound bound;

void Bound::begin() {
  MinitaurLeg::useLengths = false;
  tstart = X.t;
  front.signalState = SIGNAL_NONE;
  rear.signalState = SIGNAL_NONE;
  X.xd = 0;
  // waitingToStart = true;
  if (bAutopilot) {
    headingDes = X.yaw;
  }
  front.begin();
  delay(400);
  rear.begin();
}

bool BoundLeg::update(bool bAbsLegAngle, bool bAutopilot, uint32_t tstart, float headingDes) {
  MinitaurLeg::useLengths = false;
  // useful vars
  float ext = getPosition(EXTENSION);
  float extVel = getVelocity(EXTENSION);
  bool bFront = (legi[0]==0);

  // Vert params
  int tstance = 110;//MINITAUR
  // uint32_t tstance = 300;//MEGA
  float flightPos = 1.57;
  // const float flightRetractPos = 1.0;
  // const uint32_t tretract = 70;
  float tdPos = flightPos-0.1;
  // float kSpring = 1.8;//MEGA
#if USE_BUS
  float kSpring = 0.75;//MINI
  float kVert = 0.19;//MINI
#else
  // from old code
  // float kSpring = 0.55;//MINI
  // float kVert = 0.21;//MINI
  float kSpring = 0.55;//MINI
  float kVert = 0.19;//MINI
#endif
  // float kVert = 0.4;//MEGA
  float kOffset = 0.03;//constrain(0.05 * fabsf(X.xd), 0, 0.2);

  // Attitude control params
  float kRoll = 2.0, kRollDot = 0.02;//0.02;//MINI
  // float kRoll = 0.7, kRollDot = 0.09;// MEGA

  // FA params
  float kAngStance = 0.5;//MINI
  // float kAngStance = 1.0;//MINI WEIGHTS
  // desired standing leg angle depends on front or back
  float angDes = (bFront) ? -0.02 : 0.17;//MINITAUR
  // float angDes = (bFront) ? 0.0 : 0.2;//MEGATAUR

  // Flight params
  float kExtFltP = 0.4, kExtFltD = 0.005;//MINI
  float kAngFltP = 0.25, kAngFltD = 0.002;//MINI
  // float kExtFltP = 0.8, kExtFltD = 0.0;//MEGA
  // float kAngFltP = 1.0, kAngFltD = 0.0;//MEGA
  // Half sweep angle = arcsin(xdot * Ts / (2 rho))
  // The extension @TD is the FKext(flightPos) ~= 0.17
  float kStrideLength = tstance*0.001 / 0.36;

  // STAND PARAMS
  float kExtStand = 0.15;
  float kAngStand = 0.3;

  // SPEED DEPENDENT PARAMS
  // kVert += 0.01 * constrain(fabsf(X.xd), 0, 2);
  // float splayIncrease = 0.03 * constrain(fabsf(X.xd), 0, 2);
  // angDes += (bFront) ? -splayIncrease : splayIncrease;
  // kOffset += 0.05 * constrain(fabsf(X.xd), 0, 1);
  // tstance -= (int)(10 * constrain(fabsf(X.xd), 0, 2));

  // TEST
  if (bAutopilot) {
    // overwrite speedDes and yawDes
    // yawDes = 0.3 * fmodf_mpi_pi(X.yaw - headingDes);
    speedDes = 0;
    int t2 = X.t - tstart - 2000;
    if (t2 > 0 && t2 <= 10000) {
      speedDes = 0.00017 * t2;
    } else if (t2 > 10000) {
      speedDes = 1.7 + 0.0001 * (t2 - 10000);
    }
  }

  // test
  if (bAbsLegAngle && (mode == STANCE || mode == FLIGHT)) {
    angDes = (bFront) ? 0.12 : 0.2;//MINITAUR
    angDes -= constrain(X.pitch, -0.5, 0.5);
  }
  // TEST
  // if (!bFront && (mode == STANCE || mode == FLIGHT)) {
  //   angDes += 0.05*constrain(fabsf(X.xd), 0, 2);
  // }

  // For FA
  float speedErr = constrain(speedDes-X.xd, -0.05, 0.05);

  if (mode == STANCE) {
    // increase tstance for leap
    if (signalState == SIGNAL_LEAP_STANCE){
      tstance += (bFront) ? 50 : 60;
    }
    // normal stuff
    float frac = interpFrac(tTD, tTD + tstance, X.t);
    // float ang = atan2((ext - tdPos)*PI/tstance, extVel);
    // float phaseTerm = constrain(0.15 * latDes, -0.3, 0.3);//-0.3;
    // TEST
    float phaseTerm = phaseControlGain * arm_sin_f32(pitchdot);
    // front-back leap
    if (signalState == SIGNAL_LEAP_STANCE) {
      kRoll =20;
      kRollDot = 2;
      if (bFront) {
        if (frac > 0.2) // fraction through stance
          kOffset = 0.7; // SetOpenLoop for bFront

        angDes += 0.2; //0.26//this should be a function of speed? // adds x radians to the already calculated desired 
      } else {
         // PI/3
        // if (frac>.1){
          // angDes +=PI/6.0;
        // }
        if (frac>0.2/*0.4*/){
          kOffset = 0.5; //rear leg offset 
        }
       angDes += 0.35;
      }
    }

    float vertTerm = -kVert * arm_cos_f32(frac * PI);
    float uvert = kSpring * (flightPos - ext) + vertTerm + phaseTerm * arm_sin_f32(frac*PI) + kOffset;
    // // back-front leap: 
    // if (pendingSignal && frac>0.5)
    //   uvert += (bFront) ? 0.3 : 0.8;
    // }
    // Roll when yawing
    float rollDes = 0;//constrain(0.4 * fabsf(X.xd) * yawDes, -0.1, 0.1);
    float uroll = kRoll * (X.roll - rollDes) + kRollDot * rolldot;
    uroll = (frac > 0.3) ? constrain(uroll, -0.2, 0.2) : 0;
    setOpenLoop(EXTENSION, uvert, uroll);

    // FA: very loose spring
    angDes += asinf(kStrideLength * speedDes);
    setOpenLoop(ANGLE, kAngStance * (angDes - getPosition(ANGLE)), 0);
    // TODO implement diff controller on two legs (angles) to keep them active synced together
    
    speedAccum += getSpeed(X.pitch);

    // Liftoff
    if (X.t - tTD > tstance) {//ext > flightPos && extVel > 0) {//
      mode = FLIGHT;
      tLO = X.t;
      // LP filter this
      speed = speedAccum / (0.001*tstance*CONTROL_RATE);
      speedAccum = 0;
      if (signalState == SIGNAL_LEAP_STANCE) // advance state if not NONE
        signalState = SIGNAL_LEAP_LAND;
    }
  } else if (mode == FLIGHT) {
    if (signalState == SIGNAL_LEAP_LAND) {
      // TODO legs track absolute angle on leap land
      // if (X.t-tLO<250){
      //     flightPos -= 0.75;
      //     tdPos -= 0.95;
      //     //if(bFront) angDes  = -constrain(X.pitch,-1,1)-PI/2;
      //   }else if(X.t-tLO>=250 && X.t-tLO<300){
      //     tdPos-=0.95;
      //     angDes += (-0.15-constrain(X.pitch, -1, 1));
      //   }else{
      //     angDes += (-0.15-constrain(X.pitch, -1, 1));
      //   }
      angDes += (-0.1 -constrain(X.pitch, -1, 1));
    } else {
      // test abs leg angle flight
      // angDes += (-constrain(X.pitch, -1, 1));
    }

    setGain(EXTENSION, kExtFltP, kExtFltD);
    // setPosition(EXTENSION, (X.t - tLO > tretract) ? flightPos :  flightRetractPos, 0);
    setPosition(EXTENSION, flightPos, 0);

    // FOREAFT
    // get to the neutral point, servo around it
    if(signalState != SIGNAL_LEAP_LAND){
    angDes += asinf(kStrideLength * (-X.xd + 0.3 * speedErr));
    }
    if (X.t - tLO < 20)
      setGain(ANGLE, 0);
    else
      setGain(ANGLE, kAngFltP, kAngFltD);
    // 
    // yawDes is between +/- 1
    // FIXME: correct for body pitch, but IMU delay
    float uyaw = constrain(-yawDes /*+ 0.1 * yawdot*/, -0.3, 0.3);
    setPosition(ANGLE, angDes, uyaw);
    
    if (ext < tdPos && extVel < 0)// && X.t - tLO > tretract+20)
    { 
      mode = STANCE;
      tTD = X.t;
      if (signalState == SIGNAL_LEAP_LAND) // advance state if not NONE
        signalState = SIGNAL_NONE;
      else if (signalState == SIGNAL_QUEUE) {
        signalState = SIGNAL_LEAP_STANCE;
        // this flag is used to make the rear start its leap
        return true;
      }
    }
  } else {
    // stand
    setGain(EXTENSION, kExtStand);
    setGain(ANGLE, kAngStand);
    setPosition(ANGLE, angDes, 0);
    setPosition(EXTENSION, flightPos, 0);
    X.xd = 0;
  }
  return false;
}


