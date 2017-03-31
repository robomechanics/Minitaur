/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#ifndef MinitaurLeg_h
#define MinitaurLeg_h

#include "AbstractMotor.h"

/** @addtogroup MotorController
 *  @{
 */

enum AbstractCoord {
  EXTENSION = 0, ANGLE = 1
};

// i=0->radial, i=1->tangential

/**
 * @brief Class for coordinating two parallel coaxial motors in the Minitaur config
 * @details Link lengths can be set, and kinematics are implemented. The user can choose to use meters for the extension, or a nonlinear mapping from 0 (folded up) to pi (all the way extended)
 */
class MinitaurLeg : public AbstractMotor<2> {
public:
  /**
   * @brief If true, use the exact leg kinematics, and the extension coordinate is in length units. If false, use the mean angle as a proxy for extension (between 0 and PI).
   */
  static bool useLengths;
  /**
   * @brief Upper and lower link lengths
   */
  static float l1, l2;

  /**
   * @brief Declare the leg based on the consituent motors
   */
  MinitaurLeg(Motor *M0, Motor *M1);

  // Forward kinematics
  void physicalToAbstract(const float joints[2], float toe[2]);
  // Map forces to joint torques
  void abstractToPhysical(const float toeForce[2], float jointTorque[2]);

  // Return forward speed
  /**
   * @brief Return forward speed
   * @param bodyPitch Pitch angle of the body (required to figure out absolute leg angle)
   * @return Speed in m/s
   */
  float getSpeed(float bodyPitch);

  /**
   * @brief Get force at the toe
   * @details The lengths MinitaurLeg::l1 and MinitaurLeg::l2 need to be correct, and also Motor::setTorqueEstParams needs to have be called for this to work.
   * 
   * @param ur Radial force in N. This is an output parameter (declare before this function)
   * @param uth Tangential torque in N-m. This is an output parameter (declare before this function)
   */
  void getToeForce(float& ur, float& uth);

  /**
   * @brief Get toe forces in N in absolute coords
   * @details The lengths MinitaurLeg::l1 and MinitaurLeg::l2 need to be correct, and also Motor::setTorqueEstParams needs to have be called for this to work.
   * 
   * @param bodyPitch pitch such that > 0 when leaned forward
   * @param bRight true if this is a right leg
   * @param ux Returns horizontal force in N. It is >0 if leg is pushed "back"
   * @param uz Returns vertical force in N. It is >0 if leg is pushed "up"
   */
  void getToeForceXZ(float bodyPitch, float& ux, float& uz);

protected:
  // this does the full FK irrespective of if useLengths is false
  float FKext(float meanAng);
};

/** @} */ // end of addtogroup

#endif
