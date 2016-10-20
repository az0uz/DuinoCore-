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

#ifndef __SPIBUS_H__
#define __SPIBUS_H__

#include <Arduino.h>
#include "Bus.h"

#ifndef SPI_SPEED
    #define SPI_SPEED 1000000
#endif
#ifndef SPI_TIMEOUT_US
    #define SPI_TIMEOUT_US 1000
#endif
#ifndef SPI_LSB_FIRST
  #define SPI_LSB_FIRST 0
#endif
#ifndef SPI_LEADING_FALLING
  #define SPI_LEADING_FALLING 1
#endif
#ifndef SPI_LEADING_SETUP
  #define SPI_LEADING_SETUP 1
#endif
   
class SpiBus : public Bus
{
public:
  SpiBus(volatile uint8_t *newCsPort,
         uint8_t newCsPin);
  void begin();
  bool isConnected();
  bool cmd(uint8_t cmd);
  bool write(uint8_t reg, uint8_t val);
  uint8_t read(uint8_t reg);
  bool write(uint8_t reg, uint8_t *buffer, uint8_t len);
  bool read(uint8_t reg, uint8_t *buffer, uint8_t len);
  bool cmdRead(uint8_t cmd, uint8_t *buffer, uint8_t len);
  
private:
  void init();
  void printError(const String errorStr);
  bool waitTimeout();
  bool csConfigure();
  bool checkCs();
  bool exchangeByte(uint8_t *val);
  void select();
  void unselect();

  volatile uint8_t* const csPort;
  volatile uint8_t* const csDdr;
  const uint8_t csPin;
  bool csValide;
};

#endif //__SPIBUS_H__

