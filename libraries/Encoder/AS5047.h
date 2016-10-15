/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#ifndef AS5047_h
#define AS5047_h

#include <SPI.h>

class AS5047 {
public:
  AS5047(SPI_TypeDef *SPIx, uint8_t cs) : SPIx(SPIx), cs(cs) {}

  uint16_t init() {
    pinMode(cs, OUTPUT);
    digitalWrite(cs, HIGH);
    // encoder setup
    uint16_t res = this->read(0x3fff);
    // this is 0-4095 but ABI is 0-3999?
    return (res*4000)/(1<<14);
  }

  uint16_t cmd(uint16_t dat, bool toRead) {
    if (toRead)
      bitSet(dat, 14);
    else
      bitClear(dat, 14);
      
    dat = addPARC(dat);
    
    uint16_t res;
    
    digitalWrite(cs, LOW);
    delayMicroseconds(1);
    SPI_I2S_SendData16(SPIx, dat);
    while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET);
    res = SPI_I2S_ReceiveData16(SPIx);
    delayMicroseconds(1);
    digitalWrite(cs, HIGH);
    
    // remove parity from res by & with 0b11..11 (14 1's) = 0x3fff
    res = res & 0x3FFF;
    
    return res;
  }

  uint16_t read(uint16_t reg) {
    this->cmd(reg, true);
    return this->cmd(0, true);
  }

protected:
  SPI_TypeDef *SPIx;
  const uint8_t cs;

  uint16_t addPARC(uint16_t dat) {
    unsigned char par = 0; 
    uint16_t dat1 = dat;
    while(dat1 != 0) { 
      par = par ^ 1; 
      dat1 &= (dat1-1); // the loop will execute once for each bit of ino set
    }
    if (par == 1)
      bitSet(dat, 15);
    else
      bitClear(dat, 15);
    return dat;
  }
};

#endif
