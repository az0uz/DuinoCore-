/*
 * MIT License
 *
 * Copyright (c) 2016 Antoine de Maleprade
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef __MPU60X0_H__
#define __MPU60X0_H__

#include <Arduino.h>
#include "Bus.h"
#include "Vector.h"
#include "IMU.h"

#include "MPU60X0def.h"

#ifndef MPU60X0_RATE
  #define MPU60X0_RATE 100
#endif

#ifndef MPU60X0_DLPF
  #define MPU60X0_DLPF MPU60X0_DLPF_188
#endif

class MPU60X0 : public IMU
{
public:
  enum gyroRange_t
  {
    RANGE_250_DPS = 0,
    RANGE_500_DPS = 1,
    RANGE_1000_DPS = 2,
    RANGE_2000_DPS = 3
  };
  enum accelRange_t
  {
    RANGE_2_G = 0,
    RANGE_4_G = 1,
    RANGE_8_G = 2,
    RANGE_16_G = 3
  };
  
  MPU60X0(Bus &newBus, volatile uint8_t* newInterruptPort = 0, uint8_t newInterruptPin = 0);
  void begin();
  bool isValid() const {return (state==DATA_UPDATE);};
  bool setRange(gyroRange_t newGyroRange, accelRange_t newAccelRange);
  
  bool update();
  bool available() const {return dataAvailable;};
  const Vector<3, int16_t>& getRawAccel() const {return rawAccel;};
  const Vector<3, int16_t>& getRawGyro() const {return rawGyro;};
  int getTemperature() const {return temperature;};
  Vector<3> getAccelDefaultGain() const;
  Vector<3> getGyroDefaultGain() const;
  const float getSampleTime() const;
  
private:
  void printError(const String errorStr);
  bool checkId();
  bool config();
  bool updateData();
  bool checkInterruptPort();

  Bus &bus;
  volatile uint8_t* const interruptPort;
  volatile uint8_t* const interruptDdr;
  volatile uint8_t* const interruptPinreg;
  const uint8_t interruptPin;
  bool interruptPortValide;
  
  enum state_t
  {
    ID_CHECK,
    CONFIG_WRITE,
    DATA_UPDATE
  } state;
  unsigned long lastUpdate;
  unsigned long wait;
  
  gyroRange_t gyroRange;
  accelRange_t accelRange;
  
  bool dataAvailable;
  Vector<3, int16_t> rawAccel;
  Vector<3, int16_t> rawGyro;
  int temperature;

  enum reg_t
  {
    SMPLRT_DIV = 0x19,
    CONFIG = 0x1A,
    GYRO_CONFIG = 0x1B,
    ACCEL_CONFIG = 0x1C,
    MOT_THR = 0x1F,
    //MOT_DUR = 0x20,
    FIFO_EN = 0x23,
    I2C_MST_CTRL = 0x24,
    I2C_SLV0_ADDR = 0x25,
    I2C_SLV0_REG = 0x26,
    I2C_SLV0_CTRL = 0x27,
    I2C_SLV1_ADDR = 0x28,
    I2C_SLV1_REG = 0x29,
    I2C_SLV1_CTRL = 0x2A,
    I2C_SLV2_ADDR = 0x2B,
    I2C_SLV2_REG = 0x2C,
    I2C_SLV2_CTRL = 0x2D,
    I2C_SLV3_ADDR = 0x2E,
    I2C_SLV3_REG = 0x2F,
    I2C_SLV3_CTRL = 0x30,
    I2C_SLV4_ADDR = 0x31,
    I2C_SLV4_REG = 0x32,
    I2C_SLV4_DO = 0x33,
    I2C_SLV4_CTRL = 0x34,
    I2C_SLV4_DI = 0x35,
    I2C_MST_STATUS = 0x36,
    INT_PIN_CFG = 0x37,
    INT_ENABLE = 0x38,
    INT_STATUS = 0x3A,
    ACCEL_OUT = 0x3B, // 3x2bytes
    TEMP_OUT = 0x41, // 2bytes
    GYRO_OUT = 0x43, // 3x2bytes
    EXT_SENS_DATA = 0x49, // 24bytes
    I2C_SLV0_DO = 0x63,
    I2C_SLV0_D1 = 0x64,
    I2C_SLV0_D2 = 0x65,
    I2C_SLV0_D3 = 0x66,
    I2C_MST_DELAY_CTRL = 0x67,
    SIGNAL_PATH_RESET = 0x68,
    MOT_DETECT_CTRL = 0x69,
    USER_CTRL = 0x6A,
    PWR_MGMT_1 = 0x6B,
    PWR_MGMT_2 = 0x6C,
    FIFO_COUNT = 0x72, // 2bytes
    FIFO_R_W = 0x74,
    WHO_AM_I_REG = 0x75,
  };
  
  
};

#endif // __MPU60X0_H__



