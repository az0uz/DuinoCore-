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

#ifndef __TWIBUS_H__
#define __TWIBUS_H__

#include <Arduino.h>
#include "Bus.h"

//TWI Parameters :
#ifndef TWI_PULLUP
  #define TWI_PULLUP 1
#endif
#ifndef TWI_SPEED
  #define TWI_SPEED 400000L
#endif
#ifndef TWI_TIMEOUT_US
  #define TWI_TIMEOUT_US 1000UL
#endif

class TwiBus: public Bus
{
public:
  TwiBus();
  TwiBus(uint8_t newAddress);
  void begin();
  bool isConnected();
  //void scan();
  void setAddress(uint8_t newAddress);
  bool cmd(uint8_t cmd);
  bool write(uint8_t reg, uint8_t val);
  uint8_t read(uint8_t reg);
  bool write(uint8_t reg, uint8_t *buffer, size_t len);
  bool read(uint8_t reg, uint8_t *buffer, size_t len);
  bool cmdRead(uint8_t cmd, uint8_t *buffer, size_t len);

private:
  uint8_t address;
};


#endif //__TWIBUS_H__
