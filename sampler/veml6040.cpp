/*

The MIT License (MIT)

Copyright (c) 2015 thewknd

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

#include "Wire.h"
#ifndef __MATH_H
#include <math.h>
#endif
#include "veml6040.h"

VEML6040::VEML6040(TwoWire *wire) {
  _wire = wire;
}

bool VEML6040::begin(void) {
  bool sensorExists = false;
  // Do not call _wire->begin(); // The I2C bus is initialized in the main code
  _wire->beginTransmission(VEML6040_I2C_ADDRESS);
  if (_wire->endTransmission() == 0) {
    sensorExists = true;
  }
  return sensorExists;
}

void VEML6040::setConfiguration(uint8_t configuration) {
  _wire->beginTransmission(VEML6040_I2C_ADDRESS);
  _wire->write(COMMAND_CODE_CONF);
  _wire->write(configuration);
  _wire->write(0);
  _wire->endTransmission();
  lastConfiguration = configuration;
}

uint16_t VEML6040::read(uint8_t commandCode) {
  uint16_t data = 0;

  _wire->beginTransmission(VEML6040_I2C_ADDRESS);
  _wire->write(commandCode);
  _wire->endTransmission(false);
  _wire->requestFrom(VEML6040_I2C_ADDRESS, 2);
  while (_wire->available()) {
    data = _wire->read();
    data |= _wire->read() << 8;
  }

  return data;
}

uint16_t VEML6040::getRed(void) {
  return read(COMMAND_CODE_RED);
}

uint16_t VEML6040::getGreen(void) {
  return read(COMMAND_CODE_GREEN);
}

uint16_t VEML6040::getBlue(void) {
  return read(COMMAND_CODE_BLUE);
}

uint16_t VEML6040::getWhite(void) {
  return read(COMMAND_CODE_WHITE);
}

float VEML6040::getAmbientLight(void) {
  uint16_t sensorValue;
  float ambientLightInLux;

  sensorValue = read(COMMAND_CODE_GREEN);

  switch (lastConfiguration & 0x70) {
    case VEML6040_IT_40MS:
      ambientLightInLux = sensorValue * VEML6040_GSENS_40MS;
      break;
    case VEML6040_IT_80MS:
      ambientLightInLux = sensorValue * VEML6040_GSENS_80MS;
      break;
    case VEML6040_IT_160MS:
      ambientLightInLux = sensorValue * VEML6040_GSENS_160MS;
      break;
    case VEML6040_IT_320MS:
      ambientLightInLux = sensorValue * VEML6040_GSENS_320MS;
      break;
    case VEML6040_IT_640MS:
      ambientLightInLux = sensorValue * VEML6040_GSENS_640MS;
      break;
    case VEML6040_IT_1280MS:
      ambientLightInLux = sensorValue * VEML6040_GSENS_1280MS;
      break;
    default:
      ambientLightInLux = -1;
      break;
  }
  return ambientLightInLux;
}

uint16_t VEML6040::getCCT(float offset) {
  uint16_t red, blue, green;
  float cct, ccti;

  red = read(COMMAND_CODE_RED);
  green = read(COMMAND_CODE_GREEN);
  blue = read(COMMAND_CODE_BLUE);

  ccti = ((float)red - (float)blue) / (float)green;
  ccti = ccti + offset;
  cct = 4278.6 * pow(ccti, -1.2455);

  return (uint16_t)cct;
}
