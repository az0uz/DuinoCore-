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

#include "TwiBus.h"

enum twiRW_t
{
  WRITE = 0,
  READ = 1
};

static uint8_t twiSlaveAddress;
static twiRW_t twiRW;
static uint8_t twiSlaveReg;
static uint8_t *twiBuf;
static uint8_t twiBufIndex;
static uint8_t twiWriteSize;
static uint8_t twiReadSize;
static void twiInit();
static void twiStart();
static bool twiWaitTimeout();

TwiBus::TwiBus():
address(0)
{
}

TwiBus::TwiBus(uint8_t newAddress):
address(newAddress)
{
}

void TwiBus::begin()
{
  twiInit();
}

bool TwiBus::isConnected()
{
  uint8_t temp;
  
  twiSlaveAddress = address;
  twiRW = READ;
  twiSlaveReg = 0;
  twiBuf = &temp;
  twiBufIndex = 0;
  twiReadSize = 1;

  twiStart();
  
  return twiWaitTimeout();
}

//void TwiBus::Scan()
//{
//  
//}

void TwiBus::setAddress(uint8_t newAddress)
{
  address = newAddress;
}

bool TwiBus::cmd(uint8_t cmd){
  twiSlaveAddress = address;
  twiSlaveReg = cmd;
  twiRW = WRITE;
  twiWriteSize = 0;
  twiBufIndex = 0;

  twiStart();

  return twiWaitTimeout();
}

bool TwiBus::write(uint8_t reg, uint8_t *buffer, uint8_t len){

  twiSlaveAddress = address;
  twiRW = WRITE;
  twiSlaveReg = reg;
  twiBuf = buffer;
  twiBufIndex = 0;
  twiWriteSize = len;

  twiStart();

  return twiWaitTimeout();
}

bool TwiBus::write(uint8_t reg, uint8_t val)
{
  return write(reg, &val, 1);
}

bool TwiBus::read(uint8_t reg, uint8_t *buffer, uint8_t len){
  twiSlaveAddress = address;
  twiRW = READ;
  twiSlaveReg = reg;
  twiBuf = buffer;
  twiBufIndex = 0;
  twiReadSize = len;
    
  twiStart();
    
  return twiWaitTimeout();
}

bool TwiBus::cmdRead(uint8_t cmd, uint8_t *buffer, uint8_t len){
  return read(cmd, buffer, len);
}



uint8_t TwiBus::read(uint8_t reg)
{
  uint8_t readValue;
  if(read(reg, &readValue, 1))
  {
    return readValue;
  }else{
    return -1;
  }
}

// ********* HARDWARE FUNCTIONS ************

// ******


enum twiStatus_t
{
 TWSR_STOP = 0x00,
  
 TWSR_START = 0x08,
 TWSR_REP_START = 0x10,
 TWSR_ARB_LOST = 0x38,
 
 TWSR_MT_SLA_ACK = 0x18,
 TWSR_MT_SLA_NACK = 0x20,
 TWSR_MT_DATA_ACK = 0x28,
 TWSR_MT_DATA_NACK = 0x30,
 

 TWSR_MR_SLA_ACK = 0x40,
 TWSR_MR_SLA_NACK = 0x48,
 TWSR_MR_DATA_ACK = 0x50,
 TWSR_MR_DATA_NACK = 0x58,
 
 TWSR_ERROR = 0xFF
};

static twiStatus_t twiWaitForStatus = TWSR_STOP;

static void twiInit()
{
  static bool isInit = false;
  if(isInit) return;

  PORTD &= ~((1<<0)|(1<<1));
#if TWI_PULLUP
  PIND |= (1<<0)|(1<<1);
#endif

  TWSR = 0; //Prédiviseur = 1
  TWBR = ((F_CPU / TWI_SPEED)-16)/2;
  TWCR = 1<<TWEN;

  isInit = true;
}

static void twiStart()
{
  if((twiWaitForStatus!=TWSR_STOP) && twiWaitForStatus!=TWSR_ERROR)
  {
    twiWaitForStatus = TWSR_REP_START;
  }
  else{
    twiWaitForStatus = TWSR_START;
  }
  TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN)|(1<<TWIE);
}

static inline void twiStop()
{
  twiWaitForStatus = TWSR_STOP;
  TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO); // Send stop
}

static bool twiWaitTimeout()
{
  unsigned long startTime = micros();
  do
  {
    if(twiWaitForStatus==TWSR_ERROR) return false;
    if(twiWaitForStatus==TWSR_STOP) return true;
  }while((micros()-startTime)<TWI_TIMEOUT_US);
  Serial.println("error:twi:timeout");
  twiStop();
  TWCR = (1<<TWINT);
  return false;
}

//typedef enum twi_error_t{
//  TWI_ERR_BUSY,
//  TWI_ERR_NO_DEVICE,
//  TWI_ERR_TXRX_NACK,
//  TWI_ERR_ARB_LOST,
//  TWI_ERR_OTHER
//} 
//;

static void twiError(twiStatus_t twiStatusGiven, twiStatus_t twiStatusExpected)
{
  //  if((twiStatusExpected == TWSR_MT_SLA_ACK) &&
  //    (twiStatus == TWSR_MT_SLA_NACK))
  //  {
  //    // Slave absent
  //  }
  //  else
  //  {
  //    // Unknown twi error
  //  }
  twiStop();
  twiWaitForStatus = TWSR_ERROR;
  
  Serial.print("error:twi:");
  Serial.print(int(twiStatusGiven),HEX);
  Serial.write(',');
  Serial.println(int(twiStatusExpected),HEX);
}

static inline void twiSendSlaveAddress(uint8_t slaveAddress, twiRW_t rw)
{
  twiWaitForStatus = (rw==READ)?TWSR_MR_SLA_ACK:TWSR_MT_SLA_ACK;
  TWDR = (slaveAddress<<1)|rw;
  TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE);
}

static inline void twiSendData(uint8_t data)
{
  twiWaitForStatus = TWSR_MT_DATA_ACK;
  TWDR = data;
  TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE);
}

static inline void twiReadDataWithRepeat(bool repeat)
{
  twiWaitForStatus = repeat?TWSR_MR_DATA_ACK:TWSR_MR_DATA_NACK;
  TWCR = (1<<TWINT)|(repeat<<TWEA)|(1<<TWEN)|(1<<TWIE); 
}

ISR(TWI_vect)
{
  static bool twiRegSelected = false;
  twiStatus_t twiStatus = (twiStatus_t)(TWSR & 0xF8);
  if(twiStatus != twiWaitForStatus)
  {
    twiError(twiStatus, twiWaitForStatus);
    return;
  }
  
  switch(twiStatus)
  {
    // OFF
  case TWSR_STOP:
    break;

    // General twi state
  case TWSR_START:
    twiRegSelected = false;
  case TWSR_REP_START:
    twiSendSlaveAddress(twiSlaveAddress, (twiRegSelected)?twiRW:WRITE);
    break;
  case TWSR_ARB_LOST:
    twiError(TWSR_ARB_LOST, TWSR_ARB_LOST);
    break;

    // Master Transmitter twi state
  case TWSR_MT_SLA_ACK:
    twiSendData(twiSlaveReg);
    twiRegSelected = true;
    break;
  case TWSR_MT_SLA_NACK:
    twiError(TWSR_MT_SLA_NACK, TWSR_MT_SLA_NACK);
    break;
  case TWSR_MT_DATA_ACK:
    if(twiRW == READ)
    {
      twiStart(); // restart
    }
    else if(twiBufIndex < twiWriteSize)
    {
      twiSendData(twiBuf[twiBufIndex++]);
    }
    else{
      twiStop();
    }
    break;
  case TWSR_MT_DATA_NACK:
    twiError(TWSR_MT_DATA_NACK, TWSR_MT_DATA_NACK);
    break;

    // Master Receiver twi state
  case TWSR_MR_SLA_NACK:
    twiError(TWSR_MR_SLA_NACK, TWSR_MR_SLA_NACK);
    break;
  case TWSR_MR_DATA_ACK:
    twiBuf[twiBufIndex++] = TWDR;
    //falltrough
  case TWSR_MR_SLA_ACK:
    twiReadDataWithRepeat((twiBufIndex+1)<twiReadSize);
    break;
  case TWSR_MR_DATA_NACK:
    twiBuf[twiBufIndex++] = TWDR;
    twiStop();
    break;

  default: // unimplemented state
    twiError(TWSR_STOP, TWSR_STOP);
    break;
  }
}





