#include <Arduino.h>
#include <Wire.h>
#include <veml6040.h>

VEML6040 RGBWSensor;

void setup() {
  Serial.begin(9600);
  Wire.begin(33, 36); // SDA, SCL
  if (!RGBWSensor.begin()) {
    Serial.println("ERROR: couldn't detect the sensor");
    while (1) {}
  }
  
  /*
  * init RGBW sensor with: 
  *  - 160ms integration time
  *  - force mode with trigger
  *  - color sensor enable
  */
    
  RGBWSensor.setConfiguration(VEML6040_IT_160MS + VEML6040_TRIG_ENABLE + VEML6040_AF_FORCE + VEML6040_SD_ENABLE);	

  delay(1500);
  Serial.println("Vishay VEML6040 RGBW color sensor auto mode example");
  Serial.println("CCT: Correlated color temperature in \260K");
  Serial.println("AL: Ambient light in lux");
  delay(1000);
  Serial.println("Enter 't' or 'f' to get new color measurements");
  delay(1000);
}

void loop() {
  if (Serial.available()) {
    char inchar = Serial.read();
		
    switch (inchar) {
      case 't':   
        
        /*
        * init RGBW sensor with: 
        *  - 1280ms integration time
        *  - force mode with trigger
        *  - color sensor enable
        */
        
        RGBWSensor.setConfiguration(VEML6040_IT_1280MS + VEML6040_TRIG_ENABLE + VEML6040_AF_FORCE + VEML6040_SD_ENABLE);
        Serial.println("integration time 1280ms"); 
        break;
				
      case 'f':   
         
        /*
        * init RGBW sensor with: 
        *  - 80ms integration time
        *  - force mode with trigger
        *  - color sensor enable
        */
          
        RGBWSensor.setConfiguration(VEML6040_IT_80MS + VEML6040_TRIG_ENABLE + VEML6040_AF_FORCE + VEML6040_SD_ENABLE);
        Serial.println("integration time 80ms"); 
        break;
				
      default:
        Serial.println("Enter 't' or 'f' to get new color measurements");
        break;           
    }
  }

  Serial.print("RED: ");
  Serial.print(RGBWSensor.getRed());  
  Serial.print(" GREEN: ");
  Serial.print(RGBWSensor.getGreen());  
  Serial.print(" BLUE: ");
  Serial.print(RGBWSensor.getBlue());  
  Serial.print(" WHITE: ");
  Serial.print(RGBWSensor.getWhite()); 
  Serial.print(" CCT: ");
  Serial.print(RGBWSensor.getCCT());  
  Serial.print(" AL: ");
  Serial.println(RGBWSensor.getAmbientLight()); 
  delay(400);
}
