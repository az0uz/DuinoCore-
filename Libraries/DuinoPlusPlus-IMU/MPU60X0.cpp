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

#include "MPU60X0.h"

#if MPU60X0_DLPF==MPU60X0_DLPF_260
#define MPU60X0_GYRO_FS 8000
#else
#define MPU60X0_GYRO_FS 1000
#endif


MPU60X0::MPU60X0(Bus &newBus,
volatile uint8_t* newInterruptPort,
uint8_t newInterruptPin):
bus(newBus),
interruptPort(newInterruptPort),
interruptDdr(newInterruptPort-1),
interruptPinreg(newInterruptPort-2),
interruptPin(newInterruptPin),
interruptPortValide(false),
state(ID_CHECK),
lastUpdate(0),
wait(0),
gyroRange(RANGE_2000_DPS),
accelRange(RANGE_4_G),
dataAvailable(false)
{
  interruptPortValide = checkInterruptPort();
}

void MPU60X0::printError(const String errorStr)
{
  Serial.print("mpu:err:");
  Serial.println(errorStr);
}

void MPU60X0::begin(){
  if(!interruptPortValide)
  {
    printError("noInt");
  }else{
    (*interruptDdr) &= ~(1<<interruptPin);
    (*interruptPort) |= (1<<interruptPin); // pullup
  }
  bus.begin();
}

bool MPU60X0::checkId()
{
  uint8_t readId;
  if(!bus.read(WHO_AM_I_REG, &readId, 1))
    return false;
  return (readId==MPU60X0_ADDRESS);
}

bool MPU60X0::config()
{
  if(!bus.write(PWR_MGMT_1, MPU60X0_PLL_GX_REF))
    return false;
  if(!bus.write(SMPLRT_DIV, MPU60X0_GYRO_FS/MPU60X0_RATE-1))
    return false;
  if(!bus.write(CONFIG,  MPU60X0_DLPF))
    return false;
  if(!bus.write(INT_PIN_CFG,  0xE2)) // INT: active low, open-drain, high until status read
    return false;
  if(!bus.write(INT_ENABLE,  0x01))
    return false;
  if(!setRange(gyroRange, accelRange))
    return false;
  return true;
}

bool MPU60X0::setRange(gyroRange_t newGyroRange, accelRange_t newAccelRange)
{
  gyroRange = newGyroRange;
  accelRange = newAccelRange;
  if(!bus.write(GYRO_CONFIG,  gyroRange << 3))
    return false;
  if(!bus.write(ACCEL_CONFIG,  accelRange << 3))
    return false;
  return true;
}

const float MPU60X0::getSampleTime() const
{
	return 1.0/((float)MPU60X0_RATE);
}

Vector<3> MPU60X0::getAccelDefaultGain() const
{
  Vector<3> accelDefaultGain(9.81f*0.00006103515625f*(1<<accelRange));
  return accelDefaultGain;
}

Vector<3> MPU60X0::getGyroDefaultGain() const
{
  Vector<3> gyroDefaultGain(0.00013323124061f*(1<<gyroRange));
  return gyroDefaultGain;
}

bool MPU60X0::update()
{
  if(wait && ((micros()-lastUpdate)<wait))
    return available();
  lastUpdate = micros();

  switch(state)
  {
  case ID_CHECK:
    if(checkId()){
      state = CONFIG_WRITE;
    }
    else{
      printError("id");
      wait = 1000000UL;
    }
    break;
  case CONFIG_WRITE:
    if(config()){
      state = DATA_UPDATE;
    }else{
      printError("cfg");
      wait = 1000000UL;
    }
    break;
  case DATA_UPDATE:
    dataAvailable = updateData();
    wait = 0;
    break;
  };

  return available();
}

bool MPU60X0::updateData()
{
  if(interruptPortValide)
  {
    if(((*interruptPinreg)&(1<<interruptPin)))
    {
      
      return false;
    }
  }
  
  uint8_t status = bus.read(INT_STATUS);

  if(status&(1<<4))
  {
    printError("ovf");
  }
  
  if(!(status&0x01))
  {
    if(interruptPortValide)
      printError("int");
    return false;
  }
  
  uint8_t bytes[14];
  if(!bus.read(ACCEL_OUT, bytes, 14))
  {
    printError("dataRead");
    return false;
  }
  
  temperature = ((int8_t)bytes[6]<<8)|((int8_t)bytes[7]);
  for(int i=0; i<3; i++)
  {
    rawAccel[i] = int16_t((bytes[2*i]<<8)|(bytes[2*i+1]));
    rawGyro[i] = (int16_t)(bytes[2*i+8]<<8)|(bytes[2*i+9]);
    if(abs(rawAccel[i])>0x7000)
      printError("accSat");
    if(abs(rawGyro[i])>0x7000)
      printError("gyrSat");
  }
  if(!interruptPortValide)
  {
    wait = 1000000/MPU60X0_RATE;
  }
  return true;
}

bool MPU60X0::checkInterruptPort()
{
  bool portOk = false;
#ifdef PORTA
  if((interruptPort == &PORTA) && (interruptDdr == &DDRA) && (interruptPinreg == &PINA)) portOk = true;
#endif
#ifdef PORTB
  if((interruptPort == &PORTB) && (interruptDdr == &DDRB) && (interruptPinreg == &PINB)) {portOk = true;}
#endif
#ifdef PORTC
  if((interruptPort == &PORTC) && (interruptDdr == &DDRC) && (interruptPinreg == &PINC)) portOk = true;
#endif
#ifdef PORTD
  if((interruptPort == &PORTD) && (interruptDdr == &DDRD) && (interruptPinreg == &PIND)) portOk = true;
#endif
#ifdef PORTE
  if((interruptPort == &PORTE) && (interruptDdr == &DDRE) && (interruptPinreg == &PINE)) portOk = true;
#endif
#ifdef PORTF
  if((interruptPort == &PORTF) && (interruptDdr == &DDRF) && (interruptPinreg == &PINF)) portOk = true;
#endif
#ifdef PORTG
  if((interruptPort == &PORTG) && (interruptDdr == &DDRG) && (interruptPinreg == &PING)) portOk = true;
#endif
#ifdef PORTH
  if((interruptPort == &PORTH) && (interruptDdr == &DDRH) && (interruptPinreg == &PINH)) portOk = true;
#endif
  return portOk && (interruptPin < 8);
}
