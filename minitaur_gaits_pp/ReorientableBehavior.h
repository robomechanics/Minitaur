/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#ifndef ReorientableBehavior_h
#define ReorientableBehavior_h

#include <Behavior.h>
#include "HAL.h"

class ReorientableBehavior : public Behavior
{
  // angleFromUpright is the spherical angle of the upright vector from the 
  // inertial upright vector [0,0,1]. It is between 0, pi
  float angleFromUpright;
public:
  // Limit before which reorientating kicks in
  float ANGLE_LIMIT;
  bool bInverted;

  /**
   * @brief Depending on if inverted, returns is this leg is in the front or rear of the robot
   * 
   * @param legidx For quadruped; 0--3 
   * @return [description]
   */
  inline bool isFront(int legidx) {
    bool bFront = (legidx == 0 || legidx == 2);
    if (bInverted)
      bFront = !bFront;
    return bFront;
  }
  inline bool isRight(int legidx) {
    // doesn't flip
    return (legidx == 2 || legidx == 3);
  }

  /**
   * @brief Reorientation specific to a quadruped. This function calls the actual
   * reorientation update, and also sets bInverted. If it returns true, the 
   * behavior update() should return (legs are being controlled to reorient)
   * 
   * @return true if need to reorient, false if the behavior update() should continue
   */
  bool isReorienting() {
    float cosProd = arm_cos_f32(X.pitch)*arm_cos_f32(X.roll);
    if (cosProd >= -1 && cosProd <= 1 && isfinite(cosProd))
      angleFromUpright = acosf(cosProd);
    bool bUpright = angleFromUpright <= ANGLE_LIMIT;
    bInverted = angleFromUpright >= (PI-ANGLE_LIMIT);
    if (!bUpright && !bInverted) {
      reorientUpdate();
      return true;
    } else
      return false;
  }

  void reorientUpdate() {
    // Save user's preference
    bool currUseLengths = MinitaurLeg::useLengths;
    MinitaurLeg::useLengths = false;
    // update
    const uint8_t front[] = {0, 2};
    const uint8_t rear[] = {1, 3};
    for (int j=0; j<4; ++j) {
      leg[j].setGain(EXTENSION, 0.2);
      leg[j].setPosition(1.0);
    }
    for (int j=0; j<2; ++j) {
      // destination angle - toes point down
      float targetAngle = -X.pitch;
      if (targetAngle > HALF_PI)
        targetAngle -= TWO_PI;
      if (targetAngle < -3*HALF_PI)
        targetAngle += TWO_PI;
      // front legs
      float frontAng = map(angleFromUpright, ANGLE_LIMIT, HALF_PI, targetAngle, -HALF_PI);
      if (angleFromUpright > HALF_PI)
        frontAng = map(angleFromUpright, HALF_PI, PI-ANGLE_LIMIT, -HALF_PI, targetAngle);
      leg[front[j]].setPosition(ANGLE, frontAng);
      // rear legs
      targetAngle = -X.pitch;
      if (targetAngle > -HALF_PI)
        targetAngle -= TWO_PI;
      if (targetAngle < 3*HALF_PI)
        targetAngle += TWO_PI;
      float rearAng = map(angleFromUpright, ANGLE_LIMIT, HALF_PI, targetAngle, HALF_PI);
      if (angleFromUpright > HALF_PI)
        rearAng = map(angleFromUpright, HALF_PI, PI-ANGLE_LIMIT, HALF_PI, targetAngle);
      leg[rear[j]].setPosition(ANGLE, rearAng);
    }
    // restore user state
    MinitaurLeg::useLengths = currUseLengths;
  }

public:
  ReorientableBehavior() : angleFromUpright(0), ANGLE_LIMIT(0.7) {}
};

#endif
