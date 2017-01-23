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

#ifndef __BUS_H__
#define __BUS_H__

#include <Arduino.h>

class Bus
{
public:
  virtual void begin() = 0;
  virtual bool isConnected() = 0;
  virtual bool cmd(uint8_t cmd) = 0;
  virtual bool write(uint8_t reg, uint8_t val) = 0;
  virtual uint8_t read(uint8_t reg) = 0;
  virtual bool write(uint8_t reg, uint8_t *buffer, size_t len) = 0;
  virtual bool read(uint8_t reg, uint8_t *buffer, size_t len) = 0;
  virtual bool cmdRead(uint8_t cmd, uint8_t *buffer, size_t len) = 0;

protected:
  //~Bus();
};

#include "SpiBus.h"
#include "TwiBus.h"

#endif //__BUS_H__
