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

#include <Sonar.h>

#define SONAR_TRIGGER_PIN 6
#define SONAR_CAPTURE_PIN 8
#define DATA_SEND_INTERVAL 50

Sonar sonar(SONAR_TRIGGER_PIN, SONAR_CAPTURE_PIN);

void setup() {
  Serial.begin(9600);
  delay(2000);
  sonar.begin();
}

unsigned long nextDisplay = DATA_SEND_INTERVAL;

void loop() {
  sonar.update();
  if(millis() >= nextDisplay) {
    nextDisplay += DATA_SEND_INTERVAL;
    Serial.println(sonar.getDistance());
  }
}

  
