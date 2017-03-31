/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 * 
 * You are free to use this gait for non-commercial purposes. If you use it in
 * research, please cite 
 * - A. De and D. E. Koditschek, “Vertical hopper compositions for preflexive and feedback-stabilized quadrupedal bounding, pronking, pacing and trotting,” under review, Sep. 2016.
 */
#include "Bound.h"

// TODO:
// look at log 9079 to see how vertical the legs are staying in flight / stance
// may have to do a trial with no FA (stay vertical to see how that is working)
// absolute leg angle all the time
// uphill trials

Bound bound;

const BoundParameters minitaurBoundParams = {
  .angNom = {-0.01, 0.17}, //front/rear
  // hybrid system params
  // tstance is now only used for the FA kStrideLength estimation, not for VH
  .tstance = 110, .tdExt = 1.6,
  // stance virtual spring params
  .kSpring = 0.5, .kVert = 0.18, .kOffset = 0.0,
  // FA (or flight) params
  .kAngStance = 0.3, 
  .kExtFltP = 0.4, .kExtFltD = 0.005,
  .kAngFltP = 0.8, .kAngFltD = 0.005,
  .kNomLegRad = 0.17,
  // Attitude control params
  .kRoll = 3.0, .kRollDot = 0.01,
  // Retraction params
  .tretract = 100, .extMin = 0.6,
  // Stand params
  .kExtStand = 0.1, .kAngStand = 0.6, .kAngStandD = 0.03
};

// --------------

void Bound::begin() {
  MinitaurLeg::useLengths = false;
  // params
  params = &minitaurBoundParams;//change for other robots
  // calculated params
  kStrideLength = params->tstance*0.001 / (2*params->kNomLegRad);
  // FIXME use force for this?
  flightPos = params->tdExt + 0.1;
  // initialize state
  tstart = X.t;
  front.signalState = SIGNAL_NONE;
  rear.signalState = SIGNAL_NONE;
  X.xd = 0;//FIXME transitions? May not be zero
  // waitingToStart = true;
  // for testing
  bAutopilot = false;
  bAbsLegAngle = false;
  // antiphase: +0.1 good for weights, smaller magnitude better for no weights, 
  // but still noisy
  // phaseControlGain = 0.05;
  if (bAutopilot) {
    headingDes = X.yaw;
  }
  front.begin();
  delay(400);
  rear.begin();
}

bool BoundLeg::update() {
  // useful vars
  float ext = getPosition(EXTENSION);
  float extVel = getVelocity(EXTENSION);
  // due to possible inversion this may change
  bool bFront = pBound->isFront(legi[0]);
  const BoundParameters *p = pBound->params;
  // New absolute leg angle
  float angDes = (bFront) ? p->angNom[0] : p->angNom[1];
  if (pBound->bAbsLegAngle)
    angDes -= X.pitch;
  // phase control
  float phaseControlGain = bFront ? pBound->phaseControlGain : -pBound->phaseControlGain;

  // "Autopilot" sets speed
  if (pBound->bAutopilot) {
    // overwrite speedDes and yawDes
    // yawDes = 0.3 * fmodf_mpi_pi(X.yaw - headingDes);
    speedDes = 0;
    int t2 = X.t - pBound->tstart - 2000;
    if (t2 > 0 && t2 <= 10000) {
      speedDes = 0.00017 * t2;
    } else if (t2 > 10000) {
      speedDes = 1.7 + 0.0001 * (t2 - 10000);
    }
  }

  if (mode == STANCE) {
    // TEST
    float phaseTerm = phaseControlGain * arm_sin_f32(X.pitchdot);
    // front-back leap

    // Use vertDes to add energy (may be helpful to compensate for lower Vbatt; or do that automatically)
    float offset = p->kOffset + map(vertDes, 0.0, 1.0, 0.0, 0.3);

    // this is only approximate: the ext is in radians, not m, and also we used omega=1,
    // but at least monotonic with the correct limits
    float phase = -atan2f((ext - p->tdExt), -extVel);
    float uvert = p->kSpring * (pBound->flightPos - ext) - p->kVert * arm_cos_f32(phase) + offset + phaseTerm * arm_sin_f32(phase);
    // Roll when yawing
    float rollDes = 0;//constrain(0.4 * fabsf(X.xd) * yawDes, -0.1, 0.1);
    float uroll = p->kRoll * (X.roll - rollDes) + p->kRollDot * X.rolldot;
    uroll = (X.t - tTD > 30) ? constrain(uroll, -0.2, 0.2) : 0;
    setOpenLoop(EXTENSION, uvert, uroll);

    // setOpenLoop(ANGLE, 0, 0);
    // // FA: very loose spring
    angDes += asinf(pBound->kStrideLength * speedDes);
    setOpenLoop(ANGLE, p->kAngStance * (angDes - getPosition(ANGLE)), 0);
    // TODO implement diff controller on two legs (angles) to keep them active synced together
    
    speedAccum += getSpeed(X.pitch);

    // Liftoff
    if (ext > pBound->flightPos && extVel > 0) {//X.t - tTD > p->tstance) {
      mode = FLIGHT;
      tLO = X.t;
      // LP filter this
      speed = speedAccum / (0.001*(tLO - tTD)*CONTROL_RATE);
      speedAccum = 0;
      if (signalState == SIGNAL_LEAP_STANCE) // advance state if not NONE
        signalState = SIGNAL_LEAP_LAND;
      waitForTD = false;
    }
  } else if (mode == FLIGHT) {
    // if (signalState == SIGNAL_LEAP_LAND) {
    //   // TODO legs track absolute angle on leap land
    //   // if (X.t-tLO<250){
    //   //     flightPos -= 0.75;
    //   //     tdPos -= 0.95;
    //   //     //if(bFront) angDes  = -constrain(X.pitch,-1,1)-PI/2;
    //   //   }else if(X.t-tLO>=250 && X.t-tLO<300){
    //   //     tdPos-=0.95;
    //   //     angDes += (-0.15-constrain(X.pitch, -1, 1));
    //   //   }else{
    //   //     angDes += (-0.15-constrain(X.pitch, -1, 1));
    //   //   }
    //   angDes += (-0.1);
    // } else {
    //   // test abs leg angle flight
    //   // angDes += (-constrain(X.pitch, -1, 1));
    // }

    // TODO retraction
    // TODO differential extension for simultaneous touchdown based on body roll?
    float frac = map(X.t, tLO, tLO + p->tretract, 0.0, 1.0);
    if (frac < 0.6) {
      // go only 60% of the way through this sin, since uphill TD may be sooner
      setGain(EXTENSION, 2*p->kExtFltP, 2*p->kExtFltD);//just try double gains for retract
      setPosition(EXTENSION, pBound->flightPos - (pBound->flightPos - p->extMin) * arm_sin_f32(frac * PI), 0);
    } else {
      // waiting for touchdown (softer gains)
      setGain(EXTENSION, p->kExtFltP, p->kExtFltD);
      setPosition(EXTENSION, pBound->flightPos, 0);
      // wait for leg to extend all the way before detecting TD
      if (ext > p->tdExt + 0.001 || frac >= 1)
        waitForTD = true;
    }

    // FOREAFT
    // get to the neutral point, servo around it
    // if(signalState != SIGNAL_LEAP_LAND){
    // this gain to speedErr seems like it doesn't need to change for other robots
    angDes += asinf(pBound->kStrideLength * (-X.xd + 0.3 * pBound->speedErr));
    // }
    setGain(ANGLE, p->kAngFltP, p->kAngFltD);
    // 
    // yawDes is between +/- 1. Add some yaw damping
    float uyaw = constrain(-yawDes + 0.03 * X.yawdot, -0.3, 0.3);
    setPosition(ANGLE, angDes, uyaw);
    
    if (waitForTD && ext < p->tdExt && extVel < 0)
    { 
      mode = STANCE;
      tTD = X.t;
      // if (signalState == SIGNAL_LEAP_LAND) // advance state if not NONE
      //   signalState = SIGNAL_NONE;
      // else if (signalState == SIGNAL_QUEUE) {
      //   signalState = SIGNAL_LEAP_STANCE;
      //   // this flag is used to make the rear start its leap
      //   return true;
      // }
    }
  } else {
    // stand
    setGain(EXTENSION, p->kExtStand);
    setGain(ANGLE, p->kAngStand, p->kAngStandD);
    setPosition(ANGLE, angDes, 0);
    setPosition(EXTENSION, pBound->flightPos, 0);
    X.xd = 0;
  }
  return false;
}


