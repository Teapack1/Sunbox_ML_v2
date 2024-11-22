#ifndef VEML6040_SUNBOX_H
#define VEML6040_SUNBOX_H

#include <Wire.h>
#include <math.h>

// VEML6040 I2C ADDRESS
#define VEML6040_I2C_ADDRESS 0x10

// REGISTER CONF (00H) SETTINGS
#define VEML6040_IT_40MS    0x00
#define VEML6040_IT_80MS    0x10
#define VEML6040_IT_160MS   0x20
#define VEML6040_IT_320MS   0x30
#define VEML6040_IT_640MS   0x40
#define VEML6040_IT_1280MS  0x50

#define VEML6040_TRIG_DISABLE 0x00
#define VEML6040_TRIG_ENABLE  0x04

#define VEML6040_AF_AUTO    0x00
#define VEML6040_AF_FORCE   0x02

#define VEML6040_SD_ENABLE  0x00
#define VEML6040_SD_DISABLE 0x01

// COMMAND CODES
#define COMMAND_CODE_CONF   0x00
#define COMMAND_CODE_RED    0x08
#define COMMAND_CODE_GREEN  0x09
#define COMMAND_CODE_BLUE   0x0A
#define COMMAND_CODE_WHITE  0x0B

// G SENSITIVITY
#define VEML6040_GSENS_40MS    0.25168
#define VEML6040_GSENS_80MS    0.12584
#define VEML6040_GSENS_160MS   0.06292
#define VEML6040_GSENS_320MS   0.03146
#define VEML6040_GSENS_640MS   0.01573
#define VEML6040_GSENS_1280MS  0.007865

class VEML6040_Sunbox {
  private:
    uint16_t read(uint8_t);
    uint8_t lastConfiguration;
    TwoWire *_wire;  // Added to specify I2C bus

  public:
    VEML6040_Sunbox(TwoWire *wire = &Wire);  // Modified constructor
    bool begin(void);
    void setConfiguration(uint8_t);
    uint16_t getRed(void);
    uint16_t getGreen(void);
    uint16_t getBlue(void);
    uint16_t getWhite(void);
    uint16_t getCCT(float offset = 0.5);
    float getAmbientLight(void);
};

#endif
