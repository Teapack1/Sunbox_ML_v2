#include "veml6040_sunbox.h"

VEML6040_Sunbox::VEML6040_Sunbox(TwoWire *wire) {
  _wire = wire;
}

bool VEML6040_Sunbox::begin(void) {
  bool sensorExists = false;
  _wire->beginTransmission(VEML6040_I2C_ADDRESS);
  if (_wire->endTransmission() == 0) {
    sensorExists = true;
  }
  return sensorExists;
}

void VEML6040_Sunbox::setConfiguration(uint8_t configuration) {
  _wire->beginTransmission(VEML6040_I2C_ADDRESS);
  _wire->write(COMMAND_CODE_CONF);
  _wire->write(configuration);
  _wire->write(0);
  _wire->endTransmission();
  lastConfiguration = configuration;
}

uint16_t VEML6040_Sunbox::read(uint8_t commandCode) {
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

uint16_t VEML6040_Sunbox::getRed(void) {
  return read(COMMAND_CODE_RED);
}

uint16_t VEML6040_Sunbox::getGreen(void) {
  return read(COMMAND_CODE_GREEN);
}

uint16_t VEML6040_Sunbox::getBlue(void) {
  return read(COMMAND_CODE_BLUE);
}

uint16_t VEML6040_Sunbox::getWhite(void) {
  return read(COMMAND_CODE_WHITE);
}

float VEML6040_Sunbox::getAmbientLight(void) {
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

uint16_t VEML6040_Sunbox::getCCT(float offset) {
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
