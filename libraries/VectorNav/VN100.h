/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#ifndef VN100_h
#define VN100_h

#include <SPI.h>

typedef struct {
  uint8_t dummy[4];
  float yaw, pitch, roll;
  float iaccx, iaccy, iaccz;
  float gyrox, gyroy, gyroz;
} __attribute__ ((packed)) VN100240;

/**
 * @brief VN100 hardware interface library
 */
class VN100 {
  SPIClass& theSPI;
  uint8_t csPin;
public:
  // VN100240 vn100240pkt, vn100240pkt2;

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

    // Initial configuration
    // VPE config (register 35) defaults are 1,1,1,1 - OK
    // VPE mag config (36) set all to 0 (don't trust magnetometer)
    float magConfig[9] = {0,0,0, 0,0,0, 0,0,0};
    writeReg(36, 36, (const uint8_t *)magConfig);

    // Use DMA
    // RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    // DMA_DeInit(DMA1_Channel4);
    // DMA_DeInit(DMA1_Channel5);
    // DMA_InitTypeDef     DMA_InitStructure;
    // // DMA for reading
    // // Configure SPI_BUS RX Channel
    // DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; // From SPI to memory
    // DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI2->DR;
    // DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    // DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    // DMA_InitStructure.DMA_MemoryBaseAddr = 0; // To be set later
    // DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    // DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    // DMA_InitStructure.DMA_BufferSize = 40; // To be set later
    // DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    // DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    // DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    // DMA_Init(DMA1_Channel4, &DMA_InitStructure);

    // // Configure SPI_BUS TX Channel
    // DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST; // From memory to SPI
    // DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI2->DR;
    // DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    // DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    // DMA_InitStructure.DMA_MemoryBaseAddr = 0; // To be set later
    // DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    // DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    // DMA_InitStructure.DMA_BufferSize = 40; // To be set later
    // DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    // DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    // DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    // DMA_Init(DMA1_Channel5, &DMA_InitStructure);

    // SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, ENABLE);
    // SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);
  }

  /**
   * @brief Read a register
   * 
   * @param reg Register ID
   * @param N Size of payload packet (in bytes)
   * @param buf Pre-allocated buffer to place payload packet in
   * @return Error ID
   */
  uint8_t readReg(uint8_t reg, int N, uint8_t *buf) {
    digitalWrite(csPin, LOW);
    // delayMicroseconds(1);
    theSPI.transfer(0x01);
    theSPI.transfer(reg);
    theSPI.transfer(0x00);
    theSPI.transfer(0x00);
    // delayMicroseconds(1);
    digitalWrite(csPin, HIGH);
    delayMicroseconds(50);

    digitalWrite(csPin, LOW);
    // delayMicroseconds(1);
    uint8_t c, errId;
    for (int i=0; i<N+4; ++i) {
      c = theSPI.transfer(0x00);
      if (i==3)
        errId = c;
      else if (i >= 4) {
        buf[i-4] = c;
      }
    }
    // delayMicroseconds(1);
    digitalWrite(csPin, HIGH);
    return errId;
  }
  /**
   * @brief Write a register
   * 
   * @param reg Register ID
   * @param N Size of payload packet (in bytes)
   * @param args Payload packet
   * @return Error code in response packet (0 is good)
   */
  uint8_t writeReg(uint8_t reg, int N, const uint8_t *args) {
    digitalWrite(csPin, LOW);
    // delayMicroseconds(1);
    theSPI.transfer(0x02);
    theSPI.transfer(reg);
    theSPI.transfer(0x00);
    theSPI.transfer(0x00);
    for (int i=0; i<N; ++i)
      theSPI.transfer(args[i]);
    // delayMicroseconds(1);
    digitalWrite(csPin, HIGH);
    delayMicroseconds(50);

    digitalWrite(csPin, LOW);
    // delayMicroseconds(1);
    // "it is sufficient to just clock in only four bytes
    // on the response packet to verify that the write register took effect, 
    // which is indicated by a zero error code."
    uint8_t c, errId;
    for (int i=0; i<4; ++i) {
      c = theSPI.transfer(0x00);
      if (i==3)
        errId = c;
    }
    // delayMicroseconds(1);
    digitalWrite(csPin, HIGH);
    return errId;
  }


  // void beginRead(uint8_t reg) {
  //   digitalWrite(csPin, LOW);
  //   theSPI.transfer(0x01);
  //   theSPI.transfer(reg);
  //   theSPI.transfer(0x00);
  //   theSPI.transfer(0x00);
  //   digitalWrite(csPin, HIGH);
  //   delayMicroseconds(50);//VN100 minimum processing time=50
  // }

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
   * @param ax true inertical acc in m/s^2
   * @param ay true inertical acc in m/s^2
   * @param az true inertical acc in m/s^2
   */
  uint8_t get(float& yaw, float& pitch, float& roll, float& yawd, float& pitchd, float& rolld, float& ax, float& ay, float& az) {
    // VN100: 27 (48bytes) = YPR,MAG,ACC,ANGRATES
    // VN100: 240 (36bytes) = YPR,TRUE_INERTIAL_ACC,ANGRATES

    // NEW DMA ------------------------------

    // digitalWrite(csPin, HIGH);
    // DMA_Cmd(DMA1_Channel4, DISABLE);
    // DMA_Cmd(DMA1_Channel5, DISABLE);
    // // save data here
    // yaw = radians(vn100240pkt.yaw);
    // pitch = radians(vn100240pkt.pitch);
    // roll = radians(vn100240pkt.roll);
    // yawd = vn100240pkt.gyroz;
    // pitchd = vn100240pkt.gyroy;
    // rolld = vn100240pkt.gyrox;
    // // begin next read
    // // need small delay before asserting chip select again?
    // delayMicroseconds(80);// 20 was too small
    // beginRead(240);
    // // set up DMA for receiving data
    // // Prepare the DMA
    // digitalWrite(csPin, LOW);
    // DMA1_Channel5->CNDTR = sizeof(vn100240pkt);
    // DMA1_Channel5->CMAR = (uint32_t)&vn100240pkt2;
    // DMA1_Channel4->CNDTR = sizeof(vn100240pkt);
    // DMA1_Channel4->CMAR = (uint32_t)&vn100240pkt;
    // // Start transfer
    // SPI_ReceiveData8(SPI2);//clear rxne flag
    // DMA_Cmd(DMA1_Channel5, ENABLE);
    // DMA_Cmd(DMA1_Channel4, ENABLE);

    // OLD POLLING ----------------------

    // VN200: 73 (72 bytes) = YPR,POS,VEL,ACC_BODY,ANGRATES
    // static float dat[18];
    // readReg(73, 72, (uint8_t *)dat);
    // yaw = radians(dat[0]);
    // pitch = radians(dat[1]);
    // roll = radians(dat[2]);
    // yawd = dat[17];
    // pitchd = dat[16];
    // rolld = dat[15];

    // time polling: 350us @ clock div 4, 270us @ clock div 2
    static float dat[9];
    uint8_t errId = readReg(240, 36, (uint8_t *)dat);
    // problems with reading?
    yaw = radians(dat[0]);
    pitch = radians(dat[1]);
    roll = radians(dat[2]);
    ax = dat[3];
    ay = dat[4];
    az = dat[5];
    yawd = dat[8];
    pitchd = dat[7];
    rolld = dat[6];
    return errId;
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
  uint8_t get(float& yaw, float& pitch, float& roll, float& yawd, float& pitchd, float& rolld) {
    float ax, ay, az;// dummies
    return get(yaw, pitch, roll, yawd, pitchd, rolld, ax, ay, az);
  }
};


#endif