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

#include "Sonar.h"

Sonar::Sonar(const uint16_t newTriggerPin, const uint16_t newPulsePin):
triggerPin(newTriggerPin),
pulsePin(newPulsePin),
pulseRead(pulsePin),
lastTriggerTime(0)
{
}

void Sonar::begin()
{
  pulseRead.begin();
  pinMode(triggerPin, OUTPUT);
  digitalWrite(triggerPin, LOW);
  update();
}

void Sonar::update()
{
  unsigned long now = millis();
  if((now-lastTriggerTime) > SONAR_TRIGER_DELAY_MS)
  {
    lastTriggerTime = now;
    digitalWrite(triggerPin,HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin,LOW);
  }
}

float Sonar::getDistance()
{
  return 340.0*(float(pulseRead.getMicroseconds())/2000000.0);
}

  
