
#include "Remote.h"
#include "HAL.h"

// globals set by the remote
float speedDes = 0, yawDes = 0, latDes = 0, vertDes = 0;
uint8_t remoteKnob = 0;// 1--6
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
  speedDesF.init(0.9, CONTROL_RATE, DLPF_SMOOTH);
  yawDesF.init(0.5, CONTROL_RATE, DLPF_SMOOTH);
  enableF.init(0.5, CONTROL_RATE, DLPF_SMOOTH);
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
  float enableCmd = enableF.update(rcCmd[1]);
  throttle = (enableCmd > 6.5);

  // For bound make speed more filtered to make it easier to drive
  if (behavior == &bound)
    speedDesF.smooth = 0.999;
  else
    speedDesF.smooth = 0.999;

  float rvstick = (rcCmd[0] - REMOTE_RC_ZERO);
  float rhstick = (rcCmd[3] - REMOTE_RC_ZERO);
  // speedDes = speedDesF.update(0.5 * rvstick);//HIGH SENSITIVITY
  speedDes = speedDesF.update(0.2 * rvstick);// LOW SENSITIVITY
  // yawDes = yawDesF.update(0.05 * rhstick);
  yawDes = yawDesF.update(0.025 * rhstick);
  latDes = 0;//lhstick;

  if (REMOTE_RC_6CH) {
    vertDes = constrain(map(rcCmd[4], 5.41, 9.83, 0, 1), 0, 1);
    // knob: 6.32, 6.84, 7.37, 7.89, 8.95, 9.44 (5.16 when remote off)
    if (rcCmd[5] > 5.5 && rcCmd[5] <= 6.55) remoteKnob = 1;
    else if (rcCmd[5] > 6.55 && rcCmd[5] <= 7.1) remoteKnob = 2;
    else if (rcCmd[5] > 7.1 && rcCmd[5] <= 7.65) remoteKnob = 3;
    else if (rcCmd[5] > 7.65 && rcCmd[5] <= 8.4) remoteKnob = 4;
    else if (rcCmd[5] > 8.4 && rcCmd[5] <= 9.2) remoteKnob = 5;
    else if (rcCmd[5] > 9.2 && rcCmd[5] <= 10) remoteKnob = 6;
    else remoteKnob = 0;
  }

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

  // Cycle through behaviors
  if (REMOTE_RC_6CH && !behavior->running()) {
    if (remoteKnob > 0 && remoteKnob <= NUM_BEHAVIORS) {
      curBehavior = remoteKnob-1;
      behavior = behaviorArray[curBehavior];
    }
  }
  // signal / cycle through behaviors
  if (millis() > 2000 && fabsf(rcCmd[2] - REMOTE_RC_ZERO) > 1.3 && fabsf(rcCmd[2] - REMOTE_RC_ZERO) < 5 && millis() - lastSignal > REMOTE_SIGNAL_HYSTERESIS) {
    // the "if" above is true if the left stick horiz is pushed as a switch
    if (behavior->running()) {
      // signal
      behavior->signal();
    } else {
      if (!REMOTE_RC_6CH) {// handled by knob otherwise
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
    }
    lastSignal = millis();
  }
}

// Empty behavior for computer to set positions/openloop directly
class DoNothing : public Behavior {
public:
  void begin() {}
  void update() {}
  bool running() { return false; }
  void end() {}
  void signal() {}
};
DoNothing doNothing;

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
    // FIXME need to check checksum!
    lastRx = millis();
    // Serial1 << millis() << "\t";
    // Serial1 << _HEX(computerPacket.align) << "\t";
    // Serial1 << computerPacket.cmd << "\t";
    // Serial1 << computerPacket.param1 << "\t";
    // Serial1 << computerPacket.param2 << "\t";
    // Serial1 << computerPacket.checksum << "\n";
  }
  // 115200 baud rate => can send 115200/(8*1024)*7 = ~100 bytes in 7 ms
  if (millis() - lastTx > 9) {
    rpi.write();
    lastTx = millis();
  }
  // kill motors if cmd is kill
  enable(computerPacket.cmd != RemoteComputer::CMD_KILL);

  // Handle set_twist
  if (computerPacket.cmd!=CMD_SET_POSITION && computerPacket.cmd!=CMD_SET_OPEN_LOOP) {
    // UNICYCLE COMMANDS
    float rvstick = constrain(computerPacket.params[0], -2, 2);// m/s
    float rhstick = constrain(computerPacket.params[1], -0.5, 0.5);// bound uses between -0.25,0.25 in arbitrary units
    speedDes = speedDesF.update(rvstick);
    yawDes = yawDesF.update(rhstick);
    latDes = 0;
  }

  // Handle set_position and gait selection
  if (computerPacket.cmd==CMD_SET_POSITION) {
    behavior = &doNothing;
    for (int i=0; i<4; ++i) {
      // r0,th0,r1,th1,...,kr0,kth0,kr1,kth1,...
      leg[i].setPosition(EXTENSION, computerPacket.params[2*i]);
      leg[i].setPosition(ANGLE, computerPacket.params[2*i+1]);
      leg[i].setGain(EXTENSION, computerPacket.params[2*i+8]);
      leg[i].setGain(ANGLE, computerPacket.params[2*i+9]);
    }
  } else if (computerPacket.cmd==CMD_SET_OPEN_LOOP) {
    behavior = &doNothing;
    // TODO
  } else if (computerPacket.cmd==CMD_SET_GAIT_BOUND) {
    behavior = &bound;
  } else if (computerPacket.cmd==CMD_SET_GAIT_WALK) {
    behavior = &bound;
  } else if (computerPacket.cmd==CMD_SIGNAL0 && millis() - lastSignal > REMOTE_SIGNAL_HYSTERESIS) {
    behavior->signal();
    lastSignal = millis();
  }

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
  if (!behavior->running() && (computerPacket.cmd == RemoteComputer::CMD_START && millis() - lastRx <= RemoteComputer::TIMEOUT)) {
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

