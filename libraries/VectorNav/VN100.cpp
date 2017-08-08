/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io> 
 * and Turner Topping <turner@ghostrobotics.io>
 */
#include "VN100.h"

#define DMA_Channel_Rx     DMA1_Channel4
#define DMA_Channel_Tx     DMA1_Channel5
#define CSPIN              PB12
#define TIMER              TIMER17
#define HDR                4
#define HDR_DAT_CRC        42
#define HDR_DAT            40
#define DAT                36
#define HDR_DAT_CRC_VPE    44
#define DAT_CRC            38
#define DAT_CRC_VPE        40
#define HDR_DAT_VPE        42
#define DAT_VPE            38

uint8_t readBack;
uint8_t buffer240[HDR_DAT_CRC_VPE]; //42 = header(4) + data(36) + crc(2)
uint8_t resphead[4];
uint8_t dumWr[6];
uint8_t dumRd[HDR_DAT_CRC_VPE]; //42 = header(4) + data(36) + crc(2)
VN100240CHECKSUM global240Packet;


static inline void vn100IRQHandler(){
  digitalWrite(CSPIN, LOW);
  // DMA_Cmd(DMA_Channel_Tx, ENABLE);
  // DMA_Cmd(DMA_Channel_Rx, ENABLE);
  // SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, ENABLE);


}

static inline void afterWaitReadISR() {
  vn100IRQHandler();
  TIM_ITConfig(TIMER_MAP[TIMER17].TIMx, TIM_IT_Update, DISABLE);
  SPI_2.readDMA(HDR_DAT_CRC_VPE,buffer240,dumRd,false);
  readBack = 1;

  
  // memcpy(resphead,tempBuf,4);
  // memcpy(buffer, &tempBuf[4],Np+2);



  // DMA_Channel_Rx->CNDTR = cmdLength;
  // DMA_Channel_Rx->CMAR = (uint32_t)&_lanRxBuf[bufIdx];
  // DMA_Channel_Tx->CNDTR = cmdLength;
  //   // was already set to increment memory
  // DMA_Channel_Tx->CMAR = (uint32_t)&_lanTxBuf[bufIdx];
  //   // Clear SPIx_DR data (even though I thought it would be fine since DMA read it)
  //   // start read
  // //digitalWrite(SS, LOW);
  //   // Enable the DMAs - They will await signals from the SPI hardware
    // DMA_Cmd(DMA_Channel_Tx, DISABLE);
    // DMA_Cmd(DMA_Channel_Rx, DISABLE);
    // SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, DISABLE);
  
}
unsigned short VN100::calculateCRC(unsigned char data[], unsigned int length)
{
    unsigned int i;
    unsigned short crc = 0;
    for(i=0; i<length; i++){
      crc  = (unsigned char)(crc >> 8) | (crc << 8);
      crc ^= data[i];
      crc ^= (unsigned char) (crc & 0xff) >> 4;
      crc ^= (crc << 8) << 4;
      crc ^= ((crc & 0xff) << 4) << 1;
          }
    return crc;
}
void VN100::getVPEerrors(uint16_t vStat, uint16_t &attQaul, uint16_t &gyroSat, uint16_t &gyroRec, uint16_t &magDis, uint16_t &magSat, uint16_t &accDis, uint16_t &accSat, uint16_t &knwnMagDis, uint16_t &knwnAccDis){
    attQaul = (vStat & VN_VPE_ATTITUDE_QUAL) >> 14;
    gyroSat = (vStat & VN_VPE_GYRO_SATURATION) >> 13;
    gyroRec = (vStat & VN_VPE_GYRO_SAT_RECOVERY) >>12;
    magDis  = (vStat & VN_VPE_MAG_DISTURBANCE) >> 10;
    magSat  = (vStat & VN_VPE_MAG_SATURATION) >> 9;
    accDis  = (vStat & VN_VPE_ACC_DISTURBANCE) >> 7;
    accSat  = (vStat & VN_VPE_ACC_SATURATION) >> 6;
    knwnMagDis = (vStat & VN_VPE_KNWN_MAG_DIS) >> 4;
    knwnAccDis = (vStat & VN_VPE_KNWN_ACC_DIS) >> 3;
    Serial1<<attQaul<<' '<<gyroSat<< ' '<< gyroRec<< ' '<< magDis<< ' '<< magSat<< ' '<< accDis<< ' '<< accSat<< ' '<< knwnMagDis<< ' '<< knwnAccDis<< '\n';

    // if ((vStat & VN_VPE_ATTITUDE_QUAL) == 0xC000){ //Atitude quality field (2-bit)
      
    //   //0 - Excellent, 1 - Good, 2- Bad, 3 -Not tracking 
    //    // error code 13 - attitude not tracking
    //   //We could add Bad to this if we need to
    // // }
    // if((vStat & VN_VPE_GYRO_SATURATION) == 0x2000){ // GyroSaturation Field (1-bit)
    //   return 14; //eCode 14 - At least one gyro saturated
    // }
    // if((vStat & VN_VPE_GYRO_SAT_RECOVERY) ==0x1000){ // GyroSaturRecovery (1-bit)
    //   return 15; // Currently the gyro is recovring e
    // }
    // if((vStat & VN_VPE_MAG_DISTURBANCE) != 0x0000){ //MagDisturbance (2-bit)
    //   return 16; //Disturbance is present if not 0 (1-3 are bad)
    // }
    // if((vStat & VN_VPE_MAG_SATURATION) == 0x0200){//MagSaturation (1-bit)
    //   return 17; // at least 1 mag saturated
    // }
    // if((vStat & VN_VPE_ACC_DISTURBANCE) != 0x0000){//AccDisturbance (2-bit)
    //   if((vStat & VN_VPE_ACC_DISTURBANCE) == 0x0080){
    //     return 18; // A disturbance magnitude 1 was detected
    //   }else if((vStat & VN_VPE_ACC_DISTURBANCE) == 0x0100){
    //     return 19; // A disturbance magnitude 2 was detected
    //   }else{
    //     return 20; // A disturbance magnitude 3 was detected
    //   }
      
    // }
    // if((vStat & VN_VPE_ACC_SATURATION) == 0x0060){//AccSaturation (1-bit)
    //   return 21; // at least one acc is saturated
    // }
    // // if((vStat & VN_VPE_KNWN_MAG_DIS ) == 0x0020){ // Known magnetic distrbance (1-bit)
    // //   return 22; //Tuned out
    // // }
    // // if((vStat & VN_VPE_KNWN_ACC_DIS) == 0x0010){ // Known accel disturbance (1-bit)
    // //   return 23; //Tuned out
    // // }
    // //No Problems? 
    // return 0;
  }

  void VN100::init(uint8_t csPin) {
    this->csPin = csPin;

    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);

    _SPI.begin();
    // APB1 on F303 has prescaler 2 => 36MHz => div2 = 18MHz
    // VN100 says 16MHz max SPI speed
    _SPI.setClockDivider(SPI_CLOCK_DIV2);
    _SPI.setBitOrder(MSBFIRST);
    _SPI.setDataMode(SPI_MODE3);


    // Use DMA
    _SPI.initDMA(RCC_AHBPeriph_DMA1, DMA1_Channel5, DMA1_Channel4, DMA1_FLAG_TC5, DMA1_FLAG_TC4);
    cmd[2] = cmd[3] = 0;//other two are set each time

    // Initial configuration
    // VPE config (register 35) defaults are 1,1,1,1 - OK
    // VPE mag config (36) set all to 0 (don't trust magnetometer)
    float magConfig[9] = {.01,.01,.01, .01,.01,.01, 0.5,0.5,0.5};
    // float magConfig[9] = {1,1,1, 1,1,1, 1,1,1};
    writeReg(VN_REG_VPE_MAG_CONFIG, 36, (const uint8_t *)magConfig);
    uint8_t crcConfig[7] = {0,0,0,1,1,3,0}; // {0,0,0,1,1,3,0}
    delay(10);
    writeReg(VN_REG_COM_PRTCL_CNTRL, 7, crcConfig);
    delay(10);
    writeReg(VN_REG_COM_PRTCL_CNTRL, 7, crcConfig);
    // delay(1);
    // writeReg(VN_REG_COM_PRTCL_CNTRL, 7, crcConfig);
    // delay(1);
    // writeReg(VN_REG_COM_PRTCL_CNTRL, 7, crcConfig);
    // delay(1);
    // writeReg(VN_REG_COM_PRTCL_CNTRL, 7, crcConfig);
    // delay(1);
    // writeReg(VN_REG_COM_PRTCL_CNTRL, 7, crcConfig);
    delay(1);
      
    // writeRegCrc(VN_REG_COM_PRTCL_CNTRL, 7, crcConfig);
    // delay(1);
    // writeRegCrc(VN_REG_COM_PRTCL_CNTRL, 7, crcConfig);
    // delay(1);
    // // vn100.writeReg(6,2,testBytes);
    // delay(5);
    // writeRegCrc(VN_REG_COM_PRTCL_CNTRL, 7, crcConfig);
    // // vn100.writeReg(6,2,testBytes);
    // delay(5);
    //Write 16 bit CRC
    //B0 = 0 (Serial COunt OFF), B1 = 0 (Serial Status OFF), B2 = 0 (SPICount OFf)
    //B3 = 0 (SPIStatus OFF), B4 = 1 (Serial Checksum), B5 = 3 (SPI 16bit Checksum )
    //B6 = 0 (Send Error)
    // uint8_t crcConfig[7] = {0,0,0,0,1,3,0};
    // writeReg(VN_REG_COM_PRTCL_CNTRL,7,crcConfig);

    // Test read and request (avoid wait)
    // readReg(VN_REG_YPR_IACC_ANGR, 36, NULL, VN_REQUEST);
  }
  void VN100::OSIinit(uint8_t timer, uint8_t period){
    nvicEnable(TIMER_MAP[timer].IRQn, TIMER_IC_PRIORITY);
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    // if (timer == TIMER1 || timer == TIMER8 || timer == TIMER15 || timer == TIMER16 || timer == TIMER17)
    TIM_TimeBaseStructure.TIM_Prescaler = 71;
    // else
      // TIM_TimeBaseStructure.TIM_Prescaler = 35;
    TIM_TimeBaseStructure.TIM_Period = period-1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIMER_MAP[timer].TIMx, &TIM_TimeBaseStructure);
    TIM_Cmd(TIMER_MAP[timer].TIMx, ENABLE);
    TIM_ITConfig(TIMER_MAP[timer].TIMx, TIM_IT_Update, ENABLE);
  }


  static void OSrestart(uint8_t timer, uint8_t cspin){
    digitalWrite(cspin, HIGH);
    TIMER_MAP[timer].TIMx->CNT = 0;
    TIM_ClearITPendingBit(TIMER_MAP[timer].TIMx, TIM_IT_Update);
    TIM_ITConfig(TIMER_MAP[timer].TIMx, TIM_IT_Update, ENABLE);
    attachGPTimerUpdateInterrupt(TIMER17, afterWaitReadISR);

    //readBack=1;
  }

    /**
   * @brief Perform an asynchronous read to register 240, storing the values in the global buffers
   * 
   *
   */
  void VN100::update240Async(){
    //Request 
    cmd[0] = VN_CMD_READ;
    cmd[1] = VN_REG_YPR_IACC_ANGR;
    cmd[2] = 0;
    cmd[3] = 0; 
    //readBack=0;
    uint16_t crc = calculateCRC(cmd,4);//Calculate checksum of read command
    uint8_t *pCRC = (uint8_t*)&crc;//set up dummy for endian swap
    swapByte(&pCRC[0], &pCRC[1]); //Swap checksum bytes
    uint8_t cmdpcrc[6]; //create cmd plus crc
    memcpy(&cmdpcrc[0], &cmd, 4);//populate with cmd
    memcpy(&cmdpcrc[4], &crc, 2);//Append checksum
    //uint8_t dum[6]; //Dummy buffer to read to
    DMA_ClearITPendingBit(DMA1_IT_TC5);
    digitalWrite(csPin, LOW);
    SPI_2.readDMA(6,dumRd, cmdpcrc, false);
    // attachGPTimerUpdateInterrupt(TIMER17, afterWaitReadISR);
    // Serial1 << readBack << '\t';
    
    
  }

    /**
   * @brief Read a register
   * 
   * @param reg Register ID
   * @param N Size of payload packet (in bytes)
   * @param buf Pre-allocated buffer to place payload packet in
   * @param mode VN_REQUEST_WAIT_READ, VN_REQUEST, or VN_READ_REQUEST
   * @return Error ID
   */
  uint8_t VN100::readReg(uint8_t reg, int N, uint8_t *buf, VN100ReadMode mode=VN_REQUEST_WAIT_READ) {
    if (mode == VN_REQUEST_WAIT_READ || mode == VN_REQUEST) {
      // Request
      digitalWrite(csPin, LOW);

      cmd[0] = VN_CMD_READ;
      cmd[1] = reg;
      // _SPI.writeDMA(4, cmd); NON-crc
      uint16_t crc = calculateCRC(cmd,4);//Calculate checksum of read command
      uint8_t *pCRC = (uint8_t*)&crc;//set up dummy for endian swap
      swapByte(&pCRC[0], &pCRC[1]); //Swap checksum bytes
      uint8_t cmdpcrc[6]; //create cmd plus crc
      memcpy(&cmdpcrc[0], &cmd, 4);//populate with cmd
      memcpy(&cmdpcrc[4], &crc, 2);//Append checksum
      _SPI.writeDMA(6,cmdpcrc); //write it

      digitalWrite(csPin, HIGH);
      if (mode == VN_REQUEST_WAIT_READ) {
        // need to wait 45 ms before response (SPI overhead adds some)
        delayMicroseconds(45);
      }
      else if (mode == VN_REQUEST)
        return 0;
    }

    if (mode == VN_REQUEST_WAIT_READ || mode == VN_READ_REQUEST) {
      digitalWrite(csPin, LOW);

       // NON-CRC 
      // uint8_t tempBuf[N+4];
      // _SPI.readDMA(N+4, tempBuf);
      // memcpy(resphead, tempBuf, 4);
      // memcpy(buf, &tempBuf[4], N);
      
      uint8_t tempBuf[N+6];
      _SPI.readDMA(N+6,tempBuf);
      memcpy(resphead,tempBuf,4);
      memcpy(buf, &tempBuf[4],N+2);
          
      // delayMicroseconds(1);

      digitalWrite(csPin, HIGH);
      if (mode == VN_READ_REQUEST) {
        // Request again
        delayMicroseconds(40);
        digitalWrite(csPin, LOW);
        cmd[0] = VN_CMD_READ;
        cmd[1] = reg;
        // _SPI.writeDMA(4, cmd); NON-crc version
        uint16_t crc = calculateCRC(cmd,4);//Calculate checksum of read command
        uint8_t *pCRC = (uint8_t*)&crc;//set up dummy for endian swap
        swapByte(&pCRC[0], &pCRC[1]); //Swap checksum bytes
        uint8_t cmdpcrc[6]; //create cmd plus crc
        memcpy(&cmdpcrc[0], &cmd, 4);//populate with cmd
        memcpy(&cmdpcrc[4], &crc, 2);//Append checksum
        _SPI.writeDMA(6,cmdpcrc); //write it
        delayMicroseconds(1);//doesn't work if released too soon

        digitalWrite(csPin, HIGH);
      }
      return resphead[3];
    }
    // should never get here since mode is 0, 1, or 2
    return 0;
  }
    uint8_t VN100::writeReg(uint8_t reg, int N, const uint8_t *args) {
    digitalWrite(csPin, LOW);
    // delayMicroseconds(1);
    uint8_t tempBuf[N+4];
    tempBuf[0] = 0x02;
    tempBuf[1] = reg;
    tempBuf[2] = 0x00;
    tempBuf[3] = 0x00;
    memcpy(&tempBuf[4], args, N);
    _SPI.writeDMA(N+4, tempBuf);
    // _SPI.transfer(0x02);
    // _SPI.transfer(reg);
    // _SPI.transfer(0x00);
    // _SPI.transfer(0x00);
    // for (int i=0; i<N; ++i)
    //   _SPI.transfer(args[i]);
    delayMicroseconds(1);
    digitalWrite(csPin, HIGH);
    delayMicroseconds(50);

    digitalWrite(csPin, LOW);
    // delayMicroseconds(1);
    // "it is sufficient to just clock in only four bytes
    // on the response packet to verify that the write register took effect, 
    // which is indicated by a zero error code."
    for (int i=0; i<4; ++i) {
      resphead[i] = _SPI.transfer(0x00);
    }
    // delayMicroseconds(1);
    digitalWrite(csPin, HIGH);
    return resphead[3];
  }

  uint8_t VN100::writeRegCrc(uint8_t reg, int N, const uint8_t *args) {
    digitalWrite(csPin, LOW);
    // delayMicroseconds(1);
    uint8_t tempBuf[N+6];
    tempBuf[0] = 0x02;
    tempBuf[1] = reg;
    tempBuf[2] = 0x00;
    tempBuf[3] = 0x00;
    memcpy(&tempBuf[4], args, N);
    uint16_t crc = calculateCRC(tempBuf,N+4); //Calc crc
    uint8_t *pCRC = (uint8_t*)&crc; //create dummy for swap
    swapByte(&pCRC[0],&pCRC[1]); //Swap for endianness
    memcpy(&tempBuf[N+4],&crc,2); // append crc
    _SPI.writeDMA(N+6, tempBuf); //write
    // _SPI.transfer(0x02);
    // _SPI.transfer(reg);
    // _SPI.transfer(0x00);
    // _SPI.transfer(0x00);
    // for (int i=0; i<N; ++i)
    //   _SPI.transfer(args[i]);
    // // delayMicroseconds(1);
    digitalWrite(csPin, HIGH);
    delayMicroseconds(50);

    digitalWrite(csPin, LOW);
    // delayMicroseconds(1);
    // "it is sufficient to just clock in only four bytes
    // on the response packet to verify that the write register took effect, 
    // which is indicated by a zero error code."
    for (int i=0; i<4; ++i) {
      resphead[i] = _SPI.transfer(0x00);
    }
    // delayMicroseconds(1);
    digitalWrite(csPin, HIGH);
    return resphead[3];
  }

  void VN100::reset() {
    digitalWrite(csPin, LOW);
    _SPI.transfer(0x06);
    _SPI.transfer(0x00);
    _SPI.transfer(0x00);
    _SPI.transfer(0x00);
    digitalWrite(csPin, HIGH);
    delayMicroseconds(50);

    digitalWrite(csPin, LOW);
    // delayMicroseconds(1);
    // "it is sufficient to just clock in only four bytes
    // on the response packet to verify that the write register took effect, 
    // which is indicated by a zero error code."
    for (int i=0; i<4; ++i) {
      resphead[i] = _SPI.transfer(0x00);
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
   * @param ax true inertical acc in m/s^2
   * @param ay true inertical acc in m/s^2
   * @param az true inertical acc in m/s^2
   */
  uint8_t VN100::get(float& yaw, float& pitch, float& roll, float& yawd, float& pitchd, float& rolld, float& ax, float& ay, float& az) {
    // VN100: 27 (48bytes) = YPR,MAG,ACC,ANGRATES
    // VN100: 240 (36bytes) = YPR,TRUE_INERTIAL_ACC,ANGRATES

    // static float dat[9];
    VN100240CHECKSUM packet;
    readReg(VN_REG_YPR_IACC_ANGR, 38, (uint8_t *)&packet); // We read 38 for the data(36) and 2 for the VPEstatus(2)
    // asyncReadReg(VN_REG_YPR_IACC_ANGR, 38, (uint8_t *)&packet); // We read 38 for the data(36) and 2 for the VPEstatus(2)
    // test read and request
    // readReg(VN_REG_YPR_IACC_ANGR, 38, (uint8_t *)&packet, VN_READ_REQUEST);
    // problems with reading?
    yaw = radians(packet.dat[0]);
    pitch = radians(packet.dat[1]);
    roll = radians(packet.dat[2]);
    ax = packet.dat[3];
    ay = packet.dat[4];
    az = packet.dat[5];
    yawd = packet.dat[8];
    pitchd = packet.dat[7];
    rolld = packet.dat[6];
    uint8_t packetDat[42];
    memcpy(&packetDat[4],&packet,38);
    uint8_t refHeader[4] = {0,1,VN_REG_YPR_IACC_ANGR,0};
    memcpy(&packetDat[0],&refHeader,4);

    uint16_t crc = calculateCRC(packetDat, 42);
    uint16_t zero = 0;
    uint8_t *pCRC = (uint8_t*)&crc; //create dummy for swap
    swapByte(&pCRC[0],&pCRC[1]); //Swap for endianness
    // crcMat
    if(crc == packet.checksum){
      return 0;//getVPEerrors(packet.VPEstatus);
    }
    else{
      uint8_t crcConfig[7] = {0,0,0,1,1,3,0};
      if(zero==packet.checksum){
        writeReg(VN_REG_COM_PRTCL_CNTRL, 7, crcConfig);
        return 4;
      }else{
        return 3;  
      }
      
    }
  }

    uint8_t VN100::getWMag(float& yaw, float& pitch, float& roll, float& yawd, float& pitchd, float& rolld, float& magx, float& magy, float& magz, float& ax, float& ay, float& az) {
    // VN100: 27 (48bytes) = YPR,MAG,ACC,ANGRATES
    // VN100: 240 (36bytes) = YPR,TRUE_INERTIAL_ACC,ANGRATES

    // static float dat[9];
    VN10027CHECKSUM packet;
    readReg(VN_REG_YPR_MAG_IACC_ANGR, 50, (uint8_t *)&packet); // We read 50 for the data(48) and 2 for the VPEstatus(2)
    yaw = radians(packet.dat[0]);
    pitch = radians(packet.dat[1]);
    roll = radians(packet.dat[2]);
    magx = packet.dat[3];
    magy = packet.dat[4];
    magz = packet.dat[5];
    ax = packet.dat[6];
    ay = packet.dat[7];
    az = packet.dat[8];
    yawd = packet.dat[11];
    pitchd = packet.dat[10];
    rolld = packet.dat[9];
    uint8_t packetDat[54];
    memcpy(&packetDat[4],&packet,50);
    uint8_t refHeader[4] = {0,1,VN_REG_YPR_MAG_IACC_ANGR,0};
    memcpy(&packetDat[0],&refHeader,4);

    uint16_t crc = calculateCRC(packetDat, 54);
    uint16_t zero = 0;
    uint8_t *pCRC = (uint8_t*)&crc; //create dummy for swap
    swapByte(&pCRC[0],&pCRC[1]); //Swap for endianness
    // crcMat
    if(crc == packet.checksum){
      return 0;//getVPEerrors(packet.VPEstatus);
    }
    else{
      uint8_t crcConfig[7] = {0,0,0,1,1,3,0};
      if(zero==packet.checksum){
        writeReg(VN_REG_COM_PRTCL_CNTRL, 7, crcConfig);
        return 4;
      }else{
        return 3;  
      }
      
    }
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
  uint8_t VN100::getWMag(float& yaw, float& pitch, float& roll, float& yawd, float& pitchd, float& rolld, float& magx, float& magy, float& magz){
    //This version of get gets the magnetometer raw data, used for debugging purposes only 
    float ax, ay, az; // Dummies 
    return getWMag(yaw, pitch, roll, yawd, pitchd, rolld, magx, magy, magz, ax, ay, az);
  }
  uint8_t VN100::get(float& yaw, float& pitch, float& roll, float& yawd, float& pitchd, float& rolld) {
    float ax, ay, az;// dummies
    return get(yaw, pitch, roll, yawd, pitchd, rolld, ax, ay, az); // This is the get function for normal operation
  }
  uint8_t VN100::getGlobal(float& yaw, float& pitch, float& roll, float& yawd, float& pitchd, float& rolld){
    memcpy(resphead,buffer240,HDR);
    memcpy((uint8_t *)&global240Packet, &buffer240[HDR],DAT_CRC_VPE); // 40=data(36) + crc(2) + vpe(2)
    // float ax,ay,az;
    yaw = radians(global240Packet.dat[0]);
    pitch = radians(global240Packet.dat[1]);
    roll = radians(global240Packet.dat[2]);
    // ax = global240Packet.dat[3];
    // ay = global240Packet.dat[4];
    // az = global240Packet.dat[5];
    yawd = global240Packet.dat[8];
    pitchd = global240Packet.dat[7];
    rolld = global240Packet.dat[6];
    uint8_t packetDat[HDR_DAT_VPE];
    memcpy(&packetDat[HDR],&global240Packet,DAT_VPE); //data(36)
    uint8_t refHeader[HDR] = {0,1,VN_REG_YPR_IACC_ANGR,0};
    memcpy(&packetDat[0],&refHeader,HDR);//header (4)

    uint16_t crc = calculateCRC(packetDat, HDR_DAT_VPE); // 40 = header(4) + data(36)
    // uint16_t zero = 0;
    uint8_t *pCRC = (uint8_t*)&crc; //create dummy for swap
    swapByte(&pCRC[0],&pCRC[1]); //Swap for endianness
    // crcMat
    global240Packet.VPEstatus = (global240Packet.VPEstatus<<8) | (global240Packet.VPEstatus>>8);
    if(crc==global240Packet.checksum){
      // uint16_t attQaul,gyroSat, gyroRec, magDis, magSat, accDis, accSat, knwnMagDis, knwnAccDis;
      // getVPEerrors(global240Packet.VPEstatus,attQaul,gyroSat, gyroRec, magDis, magSat, accDis, accSat, knwnMagDis, knwnAccDis);
      
      return 0;  
    }else{
      return 1;
    }
    
    // if(crc == global240Packet.checksum){
    //   return getVPEerrors(global240Packet.VPEstatus);
    // }
    // else{
    //   uint8_t crcConfig[7] = {0,0,0,1,1,3,0};
    //   if(zero==global240Packet.checksum){
    //     writeReg(VN_REG_COM_PRTCL_CNTRL, 7, crcConfig);
    //     return 4;
    //   }else{
    //     return 3;  
    //   }
      
    // }
  };



extern "C" void DMA1_Channel5_IRQHandler() {
  if (DMA_GetITStatus(DMA1_IT_TC5)) {
    DMA_ClearITPendingBit(DMA1_IT_TC5);
    digitalWrite(CSPIN, HIGH);

  // digitalWrite(SS, HIGH);
  // do {
  //   SPI_I2S_ReceiveData16(cmdSeqSPIx);
  // } while(SPI_I2S_GetFlagStatus(cmdSeqSPIx, SPI_I2S_FLAG_RXNE));
  // while (SPI_I2S_GetFlagStatus(cmdSeqSPIx, SPI_I2S_FLAG_BSY) == SET);
    DMA_Cmd(DMA_Channel_Tx, DISABLE);
    DMA_Cmd(DMA_Channel_Rx, DISABLE);
    SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, DISABLE);
      if (readBack==0) {
        OSrestart(TIMER17,CSPIN);


      } else {  
        readBack=0;
        digitalWrite(CSPIN,HIGH);
      }
  }
}

// irqn after 50us

// extern "C" void TIM3_CC1_IRQn() {
//   //Start next step of read
//     VN100::dmaReadState = true;
//     DMA_Channel_Rx->CNDTR = cmdLength;
//     DMA_Channel_Rx->CMAR = (uint32_t)&_lanRxBuf[bufIdx];
//     DMA_Channel_Tx->CNDTR = cmdLength;
//     // was already set to increment memory
//     DMA_Channel_Tx->CMAR = (uint32_t)&_lanTxBuf[bufIdx];
//     // Clear SPIx_DR data (even though I thought it would be fine since DMA read it)
//     // start read
//     digitalWrite(SS, LOW);
//     // Enable the DMAs - They will await signals from the SPI hardware
//     DMA_Cmd(DMA_Channel_Tx, ENABLE);
//     DMA_Cmd(DMA_Channel_Rx, ENABLE);
//     SPI_I2S_DMACmd(cmdSeqSPIx, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, ENABLE);
//   }
