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

#include "PulseRead.h"

//PORTE
//#define USPIN0 (1<<6)

PulseRead::PulseRead(const uint16_t pulsePin)
{
  pinMask = digitalPinToBitMask(pulsePin);
  pinPort = digitalPinToPort(pulsePin);
  pinNum = 8;
}

bool PulseRead::begin()
{
  if(pinPort == 2){ //PORTB
    DDRB &= ~pinMask; //Input
    PORTB |= pinMask; //pullup enabled
    PCMSK0 |= pinMask; //Interrupt enabled
    PCICR = 1;  // interrupt enabled PCINT0
    for(int i=0; i<8; i++){
      if((pinMask>>i)&1){
        pinNum = i;
        break;
      }
    }
    return true;
  }else{
    Serial.println(F("Err:Pulse:wpin"));
  }
  
  return false;
  
//#ifdef PULSEREAD_USE_INT6
//  //PORTE (THROTTLE PIN)
//  DDRE &= ~USPIN0; // input
//  PORTE |= USPIN0; // pullups enabled
//  EICRB = (1 << ISC60); // interrupt on raise and fall
//  EIMSK |= (1 << INT6); // interrupt enabled INT6 (PORTE.PIN6)
//#endif
}

static uint16_t timeB0[8],timeB1[8],timeB2[8];

unsigned long PulseRead::getMicroseconds()
{
  if(pinNum>7) return 0;
  unsigned int imp1 = timeB1[pinNum]-timeB2[pinNum];
  unsigned int imp0 = timeB0[pinNum]-timeB1[pinNum];
  return min(imp0,imp1);
}

// PCINT0 interrupt vector (PORTB)
ISR(PCINT0_vect){
  unsigned int time = micros();
  static char lastPinB=0;
  
  char pin = PINB;
  char mask = (lastPinB^pin);
  lastPinB = pin;
  
  for(int i=0; i<8; i++) {
    if(mask&(1<<i)){
      timeB2[i] = timeB1[i];
      timeB1[i] = timeB0[i];
      timeB0[i] = time;
    }
  }
}

//#ifdef PULSEREAD_USE_INT6
//ISR(INT6_vect){
//  unsigned int time = micros();
//  time2[0] = time1[0];
//  time1[0] = time0[0];
//  time0[0] = time;
//}
//#endif

