/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#ifndef VN100_h
#define VN100_h

#include <SPI.h>

/**
 * @brief VN100 hardware interface library
 */
class VN100 {
public:
  SPIClass& theSPI;
  uint8_t csPin;

  /**
   * @brief Construct with reference to which SPI
   * @param theSPI SPI, SPI_2, etc.
   */
  VN100(SPIClass& theSPI) : theSPI(theSPI) {}
  
  /**
   * @brief Send initialization commands to the VN100
   * 
   * @param csPin chip select pin
   */
  void init(uint8_t csPin) {
    this->csPin = csPin;

    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);

    theSPI.begin();
    // APB1 on F303 has prescaler 2 => 36MHz
    // VN100 says 16MHz max SPI speed
    theSPI.setClockDivider(SPI_CLOCK_DIV2);
    theSPI.setBitOrder(MSBFIRST);
    theSPI.setDataMode(SPI_MODE3);
  }

  void readReg(uint8_t reg, int N, uint8_t *buf) {
    digitalWrite(csPin, LOW);
    // delayMicroseconds(1);
    theSPI.transfer(0x01);
    theSPI.transfer(reg);
    theSPI.transfer(0x00);
    theSPI.transfer(0x00);
    // delayMicroseconds(1);
    digitalWrite(csPin, HIGH);
    delayMicroseconds(100);

    digitalWrite(csPin, LOW);
    // delayMicroseconds(1);
    for (int i=0; i<N+4; ++i) {
      uint8_t c = theSPI.transfer(0x00);
      // if (i==3 && c!= 0)
      if (i >= 4) {
        buf[i-4] = c;
      }
    }
    // delayMicroseconds(1);
    digitalWrite(csPin, HIGH);
  }
  void writeReg(uint8_t reg, int N, const uint8_t *args) {
    digitalWrite(csPin, LOW);
    // delayMicroseconds(1);
    theSPI.transfer(0x01);
    theSPI.transfer(reg);
    theSPI.transfer(0x00);
    theSPI.transfer(0x00);
    for (int i=0; i<N; ++i)
      theSPI.transfer(args[i]);
    // delayMicroseconds(1);
    digitalWrite(csPin, HIGH);
    delayMicroseconds(100);

    digitalWrite(csPin, LOW);
    // delayMicroseconds(1);
    for (int i=0; i<N+4; ++i) {
      theSPI.transfer(0x00);
      // if (i==3 && c!= 0)
      // if (i >= 4) {
      //   buf[i-4] = c;
      // }
    }
    // delayMicroseconds(1);
    digitalWrite(csPin, HIGH);
  }

  /**
   * @brief Retrieves angles and angular rates
   * @details This function blocks for a few hundred microseconds.
   * 
   * @param yaw in radians
   * @param pitch in radians
   * @param roll in radians
   * @param yawd in rad/s
   * @param pitchd in rad/s
   * @param rolld in rad/s
   */
  void get(float& yaw, float& pitch, float& roll, float& yawd, float& pitchd, float& rolld) {

    // VN200: 73 (72 bytes) = YPR,POS,VEL,ACC_BODY,ANGRATES
    // static float dat[18];
    // readReg(73, 72, (uint8_t *)dat);
    // yaw = radians(dat[0]);
    // pitch = radians(dat[1]);
    // roll = radians(dat[2]);
    // yawd = dat[17];
    // pitchd = dat[16];
    // rolld = dat[15];

    // VN100: 27 (48bytes) = YPR,MAG,ACC,ANGRATES
    // VN100: 240 (36bytes) = YPR,TRUE_INERTIAL_ACC,ANGRATES

    // time polling: 350us @ clock div 4, 270us @ clock div 2
    static float dat[12];
    readReg(240, 36, (uint8_t *)dat);
    yaw = radians(dat[0]);
    pitch = radians(dat[1]);
    roll = radians(dat[2]);
    yawd = dat[8];
    pitchd = dat[7];
    rolld = dat[6];

    // OLD
    // takes about 400us polling
    // float dat[3];
    // readReg(8, 12, (uint8_t *)dat);
    // yaw = radians(dat[0]);
    // pitch = radians(dat[1]);
    // roll = radians(dat[2]);
    // delayMicroseconds(30);
    // readReg(19, 12, (uint8_t *)dat);
    // yawd = dat[2];
    // pitchd = dat[1];
    // rolld = dat[0];
  }
};


#endif