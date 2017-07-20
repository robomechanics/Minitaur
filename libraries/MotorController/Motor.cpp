/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#include "Motor.h"

// ===============================================================================
// Base class
// ===============================================================================

// Static members - can be overridden
int Motor::updateRate = 1000;
float Motor::velSmooth = 0.8;
float Motor::rpsLimit = 500; // speed limit in radians per second

void Motor::init(float zero, int8_t direction, float gearRatio) {
  this->zero = zero;
  // check
  if (direction != -1 && direction != 1)
    direction = 1;
  this->direction = direction;

  this->gearRatio = gearRatio;
  this->unwrapOffset = 0;
  this->prevPosition = NAN;
  this->posLimit = Motor::rpsLimit/float(Motor::updateRate);
  this->maxAmplitude = 1.0;

  mode = OPEN_LOOP_MODE;
  val = correctedVal = setpoint = 0;
  driverDirection = 1;
  barrier.init();
  enableFlag = false; // Only matters for BLCon v3.4
  bContinuousRotation = true;
  pd.init(Motor::velSmooth, Motor::updateRate, DLPF_ANGRATE);

  // NEW testing overheating protection
  tempProxy=0;
}

void Motor::setBarrier(float ll, float ul) {
  barrier.ll = ll;
  barrier.ul = ul;
  barrier.enabled = true;
}

void Motor::resetOffset() {
  unwrapOffset = 0;
  prevPosition = NAN;
}

float Motor::getPosition() {
  // 1. compensate for zero BEFORE direction, gearRatio
  curPos = bContinuousRotation ? fmodf_mpi_pi(getRawPosition()-zero) : (getRawPosition()-zero);
  // 2. "unwrap" based on gearRatio. To do this, need to keep track of the previous position and check for wrap-arounds. initially unwrapOffset=0
  // check for a jump in position, and count number of wrap-arounds in unwrapOffset
  if (!isnan(prevPosition) && fabsf(curPos - prevPosition) > PI) {
    unwrapOffset += (curPos < prevPosition) ? 1 : -1;
  }
  prevPosition = curPos;
  // 3. compensate for direction and gearRatio
  if (bContinuousRotation)
    return fmodf_mpi_pi((TWO_PI*unwrapOffset + curPos) * direction / gearRatio);
  else
    return (TWO_PI*unwrapOffset + curPos) * direction / gearRatio;
}

void Motor::setGain(float Kp, float Kd) {
  pd.setGain(Kp, Kd);
}

void Motor::setOpenLoop(float val) {
  this->mode = OPEN_LOOP_MODE;
  this->val = val;
}

void Motor::setPosition(float setpoint) {
  this->mode = POSITION_MODE;
  this->setpoint = setpoint;
}


void Motor::setTorqueEstParams(float Kt, float res, float Vsource, float curLim) {
  torqueFactor = gearRatio * Kt * Vsource/res;
  if (curLim > 0)
    maxAmplitude = constrain(curLim * res / Vsource, 0, 1);
}

float Motor::mapVal(float val) {
  float newVal = constrain(val * direction * driverDirection, -maxAmplitude, maxAmplitude);

  // motor overheating protection--these will turn into params
  // const float ctsAmpl = 0.3;
  // const float heatEstSmooth = 0.99995;
  // if (tempProxy > ctsAmpl && fabsf(correctedVal) > ctsAmpl) {
  //   newVal = newVal/fabsf(correctedVal) * ctsAmpl;
  // }
  // tempProxy = heatEstSmooth * tempProxy + (1-heatEstSmooth)/ctsAmpl*newVal*newVal;

  return newVal;
}

float Motor::update() {
  float pos = getPosition();

  // Velocity calculation should happen independent of mode
  // float error = (bContinuousRotation) ? fmodf_mpi_pi(pos - setpoint) : (pos - setpoint);
  // FIXME bContinuousRotation
  float posCtrlVal = pd.update(pos, setpoint);

  // In position mode, update the motor command
  if (mode == POSITION_MODE)
    val = posCtrlVal;
  // In open-loop mode, val has been set and nothing else needs to be done

  // Send command, but don't modify "val" (set by user)
  correctedVal = mapVal(val);// + barrier.calculate(pos));

  // send the command
  sendOpenLoop(correctedVal);

  // Return the command so that the slave can do the same thing
  return correctedVal;
}

// ===============================================================================
// Derived classes: driver / feedback device specific
// ===============================================================================

bool BlCon34::useEXTI = false;

void BlCon34::initCommon(uint8_t outPin_, float zero, int8_t dir, float gearRatio) {
  pinMode(outPin_, PWM);
  this->outPin = outPin_;
  analogWriteFloat(outPin, 0);
  driverDirection = 1;
  // Call base class init
  Motor::init(zero, dir, gearRatio);
}

void BlCon34::init(uint8_t outPin_, uint8_t inPin_, float zero, int8_t dir, float gearRatio) {
  // usePwmIn = true;
  // pwmIn(inPin_);
  if (BlCon34::useEXTI)
    pinMode(inPin_, PWM_IN_EXTI);
  else
    pinMode(inPin_, PWM_IN);
  this->inPin = inPin_;
  initCommon(outPin_, zero, dir, gearRatio);
}

// void BlCon34::init(uint8_t outPin_, uint8_t inPin_, float zero, int8_t dir, float gearRatio) {
//   // usePwmIn = false;
//   // pinModePulseIn(inPin_);
//   pinMode(inPin_, PWM_IN);
//   this->inPin = inPin_.pin;
//   initCommon(outPin_, zero, dir, gearRatio);
// }

void BlCon34::enable(bool flag) {
  enableFlag = flag;
}

float BlCon34::getRawPosition() {
  float inp = pwmIn(inPin);
  if (inp == 0.0) {
    // try to avoid read errors where pwmIn returns 0
    return prevPos;
  }
  else {
    prevPos = map(inp, 0.1, 0.9, 0, TWO_PI);
    return prevPos;
  }
}

void BlCon34::sendOpenLoop(float val) {
  analogWriteFloat(outPin, (enableFlag) ? map(val, -1, 1, 0.12, 0.88) : 0);
}
