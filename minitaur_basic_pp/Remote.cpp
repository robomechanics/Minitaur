/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#include "Remote.h"
#include "HAL.h"

// globals set by the remote
float speedDes = 0, yawDes = 0, latDes = 0;
RemoteRC remoteRC;
RemoteComputer remoteComputer;

// locals
int curBehavior = 0;
Behavior *behavior = behaviorArray[curBehavior];

// function to programmatically change behavior
void activateBehavior(Behavior *behav) {
  behavior = behav;
  behavior->begin();
}


void RemoteRC::begin() {
  // RC receiver
  for (int i=0; i<NRECPINS; ++i)
    pinMode(rcRecPin[i], PWM_IN_EXTI);

  // Low pass user desired speed
  speedDesF.init(0.9995, CONTROL_RATE, DLPF_SMOOTH);
  // speedDesF.init(0.9, CONTROL_RATE, DLPF_SMOOTH);
  yawDesF.init(0.5, CONTROL_RATE, DLPF_SMOOTH);
}

void RemoteRC::updateInterrupt() {
  // RC receiver
  for (int i=0; i<NRECPINS; ++i) {
    int period=0, pulsewidth=0;
    pwmInRaw(rcRecPin[i], &period, &pulsewidth);
    // no signal if transmitter is off
    float dummy = (period > 0) ? pulsewidth*100/((float)period) : 0;
    // some basic error checking
    if (dummy > 5 && dummy < 10)
      rcCmd[i] = dummy;
  }

  // this depends on the position of the "throttle", so setting that before flipping the switch could select different behaviors?
  throttle = (rcCmd[1] > 6.5 && rcCmd[1] < 15);

  float rvstick = (rcCmd[0] - REMOTE_RC_ZERO);
  float rhstick = (rcCmd[3] - REMOTE_RC_ZERO);
  // HIGH SENSITIVITY
  // speedDes = speedDesF.update(0.5 * rvstick);
  // yawDes = yawDesF.update(0.1 * rhstick);
  // LOW SENSITIVITY
  speedDes = speedDesF.update(0.2 * rvstick);
  yawDes = yawDesF.update(0.05 * rhstick);
  latDes = 0;//lhstick;
  // end behavior
  if (behavior->running() && !throttle) {
    behavior->end();
    digitalWrite(led1, HIGH);
    openLog.enable(false);
  }
}

void RemoteRC::updateLoop() {
  // start behavior
  if (!behavior->running() && throttle) {
    digitalWrite(led1, LOW);
    behavior->begin();
    openLog.enable(true);
  }
  if (millis() > 2000 && fabsf(rcCmd[2] - REMOTE_RC_ZERO) > 1.5 && fabsf(rcCmd[2] - REMOTE_RC_ZERO) < 5 && millis() - lastSignal > REMOTE_SIGNAL_HYSTERESIS) {
    if (behavior->running()) {
      // signal
      behavior->signal();
    } else {
      // Cycle through behaviors
      halHeartbeatEnabled = false;
      curBehavior = (rcCmd[2] > REMOTE_RC_ZERO) ? curBehavior+1 : curBehavior+NUM_BEHAVIORS-1;
      curBehavior = curBehavior % NUM_BEHAVIORS;
      for (uint8_t i=0; i<2*(curBehavior+1); ++i) {
        digitalWrite(led1, TOGGLE);
        digitalWrite(led0, TOGGLE);
        delay(25);
      }
      behavior = behaviorArray[curBehavior];
      halHeartbeatEnabled = true;
    }
    lastSignal = millis();
  }
}

void RemoteComputer::begin() {
  // raspi
  rpi.begin(115200, sizeof(X), (void *)&X, sizeof(computerPacket));
  rpi.enable(true);// always send data

  // Low pass user desired speed
  speedDesF.init(0.9995, CONTROL_RATE, DLPF_SMOOTH);
  yawDesF.init(0.5, CONTROL_RATE, DLPF_SMOOTH);

  behavior = &bound;
}

void RemoteComputer::updateInterrupt() {
  // receive
  int res = rpi.received(0xbbaa, (uint8_t *)&computerPacket);
  if (res) {
    lastRx = millis();
    // Serial1 << millis() << "\t";
    // Serial1 << _HEX(computerPacket.align) << "\t";
    // Serial1 << computerPacket.cmd << "\t";
    // Serial1 << computerPacket.param1 << "\t";
    // Serial1 << computerPacket.param2 << "\t";
    // Serial1 << computerPacket.checksum << "\n";
  }
  if (millis() - lastTx > 9) {
    rpi.write();
    lastTx = millis();
  }

  float rvstick = constrain(computerPacket.param1, -2, 2);// m/s
  float rhstick = constrain(computerPacket.param2, -0.5, 0.5);// bound uses between -0.25,0.25 in arbitrary units
  speedDes = speedDesF.update(rvstick);
  yawDes = yawDesF.update(rhstick);
  latDes = 0;
  // kill motors if cmd is kill
  enable(computerPacket.cmd != RemoteComputer::CMD_KILL);
  // log when bounding
  openLog.enable(behavior->running());
  // end behavior
  if (behavior->running() && (computerPacket.cmd == RemoteComputer::CMD_STAND || millis() - lastRx > RemoteComputer::TIMEOUT)) {
    behavior->end();
    digitalWrite(led1, HIGH);
  }
}

void RemoteComputer::updateLoop() {
  // start behavior
  if (!behavior->running() && (computerPacket.cmd == RemoteComputer::CMD_BOUND && millis() - lastRx <= RemoteComputer::TIMEOUT)) {
    // TODO switch computerPacket.cmd to select behavior
    digitalWrite(led1, LOW);
    behavior->begin();
    enable(true);
  }
}

// // Nunchuck code

// void RemoteNunchuck::begin() {
//   nunchuck.begin(&Wire, PF6);
// }

// void RemoteNunchuck::updateInterrupt(Behavior *behavior) {}

// void RemoteNunchuck::updateLoop(Behavior *behavior) {
//   nunchuck.update();
//   if (nunchuck.isPaired) {
//     // begin/end
//     if (nunchuck.buttonC && !behavior->running()) {
//       openLog.enable(true);
//       behavior->begin();
//       digitalWrite(led1, LOW);
//       lastCPress = millis();
//     }
//     if (nunchuck.buttonZ && millis() - lastZPress > 500) {
//       if (behavior->running()) {
//         openLog.enable(false);
//         behavior->end();
//         digitalWrite(led1, HIGH);
//       } else {
//         // // CYCLE THROUGH BEHAVIORS
//         // curBehavior = (curBehavior+1)%NUM_BEHAVIORS;
//         // for (uint8_t i=0; i<curBehavior+1; ++i) {
//         //   digitalWrite(led, LOW);
//         //   delay(50);
//         //   digitalWrite(led, HIGH);
//         //   delay(100);
//         // }
//         // behavior = behaviorArray[curBehavior];
//       }
//       lastZPress = millis();
//     }

//     // turning
//     yawDes = -nunchuck.joy[0];

//     // speed
//     // secret high scale if C button held down, tame settings otherwise
//     // float speedScale = (nunchuck.buttonC && behavior->running()) ? 2.5 : 0.25;

//     // Fixed speedScale, C button for jump
//     float speedScale = 1.2;
//     // float speedScale = 0.25;
//     if (behavior->running() && nunchuck.buttonC && millis() - lastCPress > 1000) {
//       behavior->signal();
//       lastCPress = millis();
//     }

//     speedDes = speedDesF.update(nunchuck.joy[1] * speedScale);
//   }
// }

