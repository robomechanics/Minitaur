/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#ifndef SoftStart_h
#define SoftStart_h

#include "HAL.h"

class SoftStart {
  bool bPwrOnStateObtained;
  uint32_t tPwrOnAnim0, tPwrOnAnimEnd;
  float pwrOnExt[4], pwrOnAng[4];

public:
  SoftStart() : bPwrOnStateObtained(false), tPwrOnAnim0(0), tPwrOnAnimEnd(2000) {}
  void update(float behavExtDes) {
    if (!bPwrOnStateObtained) {
      for (int i=0; i<4; ++i) {
        pwrOnAng[i] = leg[i].getPosition(ANGLE);
        pwrOnExt[i] = leg[i].getPosition(EXTENSION);
      }
      bPwrOnStateObtained = true;
      tPwrOnAnim0 = X.t;
    }
    for (int i=0; i<4; ++i) {
      bool bFront = (i==0) || (i==2);
      leg[i].setGain(EXTENSION, 0.2);
      leg[i].setGain(ANGLE, 0.4);
      if (X.t < 1500)
        leg[i].setGain(ANGLE, 0.05);
      // these two depend on the behavior: could make the behavior return these
      // for now these are reasonable for bound
      float behavAngDes = 0.0;
      leg[i].setPosition(EXTENSION, map(constrain(X.t,0,tPwrOnAnimEnd),tPwrOnAnim0,tPwrOnAnimEnd,pwrOnExt[i],behavExtDes));
      float ang0 = pwrOnAng[i];
      // avoid intersecting the body
      if (bFront && pwrOnAng[i] > HALF_PI)
        ang0 -= TWO_PI;
      else if (!bFront && pwrOnAng[i] < -HALF_PI)
        ang0 += TWO_PI;
      leg[i].setPosition(ANGLE, map(constrain(X.t,0,tPwrOnAnimEnd),tPwrOnAnim0,tPwrOnAnimEnd,ang0,behavAngDes));
    }
  }

  bool running() {
    return (X.t < tPwrOnAnimEnd);
  }
};

#endif
