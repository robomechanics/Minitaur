/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io> 
 * and Turner Topping <turner@ghostrobotics.io>
 */
#ifndef VN_h
#define VN_h

#include <SPI.h>
#include "nvic.h"

#define VN_CMD_READ              0x01
#define VN_CMD_WRITE             0x02
#define VN_REG_VPE_MAG_CONFIG    36
#define VN_REG_YPR_IACC_ANGR     240
#define VN_REG_COM_PRTCL_CNTRL   30
#define VN_REG_YPR_MAG_IACC_ANGR 27

#define VN_VPE_ATTITUDE_QUAL     0xC000
#define VN_VPE_GYRO_SATURATION   0x2000
#define VN_VPE_GYRO_SAT_RECOVERY 0x1000
#define VN_VPE_MAG_DISTURBANCE   0x0C00
#define VN_VPE_MAG_SATURATION    0x0200
#define VN_VPE_ACC_DISTURBANCE   0x0180
#define VN_VPE_ACC_SATURATION    0x0060
#define VN_VPE_KNWN_MAG_DIS      0x0010
#define VN_VPE_KNWN_ACC_DIS      0x0008

enum VN100ReadMode {
  VN_REQUEST_WAIT_READ, VN_REQUEST, VN_READ_REQUEST
};

typedef struct {
  float dat[9];
  uint16_t VPEstatus;
  uint16_t checksum;

  
} __attribute__ ((packed)) VN100240CHECKSUM;

typedef struct {
  float dat[12];
  //uint16_t VPEstatus;
  uint16_t checksum;
  
} __attribute__ ((packed)) VN10027CHECKSUM;
/**
 * @brief VN100 hardware interface library
 */
class VN100 {
  SPIClass& _SPI;
  uint8_t csPin;
  uint8_t cmd[4];

  // Calculates the 16-bit CRC for the given ASCII or binary message.
  
  void getVPEerrors(uint16_t vStat, uint16_t &attQaul, uint16_t &gyroSat, uint16_t &gyroRec, uint16_t &magDis, uint16_t &magSat, uint16_t &accDis, uint16_t &accSat, uint16_t &knwnMagDis, uint16_t &knwnAccDis);
  
public:
  // response header for SPI read/write commands
  
  //uint8_t resphead[4];
  VN100(SPIClass& _SPI) : _SPI(_SPI) {}

  unsigned short calculateCRC(unsigned char data[], unsigned int length);
  void init(uint8_t csPin);
  void OSIinit(uint8_t timer, uint8_t period);
  void OSrestart(uint8_t timer, uint8_t cspin);
  void update240Async();
  uint8_t readReg(uint8_t reg, int N, uint8_t *buf, VN100ReadMode mode);
  uint8_t writeReg(uint8_t reg, int N, const uint8_t *args);
  uint8_t writeRegCrc(uint8_t reg, int N, const uint8_t *args);
  void reset();
  uint8_t get(float& yaw, float& pitch, float& roll, float& yawd, float& pitchd, float& rolld, float& ax, float& ay, float& az);
  uint8_t getWMag(float& yaw, float& pitch, float& roll, float& magx, float& magy, float& magz, float& yawd, float& pitchd, float& rolld, float& ax, float& ay, float& az);
  uint8_t getWMag(float& yaw, float& pitch, float& roll, float& yawd, float& pitchd, float& rolld, float& magx, float& magy, float& magz);
  uint8_t get(float& yaw, float& pitch, float& roll, float& yawd, float& pitchd, float& rolld);
  uint8_t getGlobal(float& yaw, float& pitch, float& roll, float& yawd, float& pitchd, float& rolld);
};


#endif