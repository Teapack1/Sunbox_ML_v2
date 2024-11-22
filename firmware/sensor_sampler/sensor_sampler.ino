#include <veml6040_sunbox.h>
#include <Wire.h>
#include <Adafruit_VCNL4040.h>
#include <driver/ledc.h>  // Necessary for PWM handling

// Define PWM settings
const int pwmFreq = 10000;    // Frequency of PWM
const int pwmResolution = 8; // 8-bit resolution (0-1024)

// Update PWM pins to output-capable GPIOs
const int pinCW = 32;        // GPIO for Cool White LED
const int pinWW = 35;        // GPIO for Warm White LED
const int distLED = 16;

// Define two I2C buses
TwoWire I2C_0 = TwoWire(0);  // I2C bus for U1 and LS-B (Pins SDA_0=21, SCL_0=22)
TwoWire I2C_1 = TwoWire(1);  // I2C bus for LS-A (Pins SDA_1=25, SCL_1=26)

// Create instances of the sensors
Adafruit_VCNL4040 vcnl4040;    // Sensor U1
VEML6040_Sunbox veml6040_LSB(&I2C_0); // Sensor LS-B on I2C_0
VEML6040_Sunbox veml6040_LSA(&I2C_1); // Sensor LS-A on I2C_1

void setup() {
   pinMode(distLED, OUTPUT);
  digitalWrite(distLED, LOW);
  
  Serial.begin(115200);

  // Initialize I2C buses
  I2C_0.begin(21, 22);  // SDA_0, SCL_0
  I2C_1.begin(25, 26);  // SDA_1, SCL_1

  // Set I2C clock speed
  I2C_0.setClock(100000);
  I2C_1.setClock(100000);

  // Adding delay to ensure bus stability
  delay(500);

  // Initialize VCNL4040 sensor on I2C_0
  Serial.println("Initializing VCNL4040 (U1) sensor...");
  if (!vcnl4040.begin(VCNL4040_I2CADDR_DEFAULT, &I2C_0)) {
    Serial.println("Failed to initialize VCNL4040 (U1) sensor! Please check your wiring.");
    while (1);
  }
  Serial.println("VCNL4040 (U1) initialized!");

  // Initialize VEML6040 sensor LS-B on I2C_0
  Serial.println("Initializing VEML6040 (LS-B) sensor...");
  if (!veml6040_LSB.begin()) {
    Serial.println("Failed to initialize VEML6040 (LS-B) sensor! Please check your wiring.");
    while (1);
  }
  Serial.println("VEML6040 (LS-B) initialized!");

  // Set configuration for LS-B
  veml6040_LSB.setConfiguration(VEML6040_IT_40MS | VEML6040_TRIG_DISABLE | VEML6040_AF_AUTO | VEML6040_SD_ENABLE);

  // Initialize VEML6040 sensor LS-A on I2C_1
  Serial.println("Initializing VEML6040 (LS-A) sensor...");
  if (!veml6040_LSA.begin()) {
    Serial.println("Failed to initialize VEML6040 (LS-A) sensor! Please check your wiring.");
    while (1);
  }
  Serial.println("VEML6040 (LS-A) initialized!");

  // Set configuration for LS-A
  veml6040_LSA.setConfiguration(VEML6040_IT_40MS | VEML6040_TRIG_DISABLE | VEML6040_AF_AUTO | VEML6040_SD_ENABLE);

  // Set up PWM for Cool White and Warm White LEDs using updated ledc functions
  ledcAttach(pinCW, pwmFreq, pwmResolution);
  ledcAttach(pinWW, pwmFreq, pwmResolution);
}

void loop() {
  
  // Increase control_value from 0.000 to 1.000
  for (float control_value = 0.000; control_value <= 1.000; control_value += 0.001) {
    // Map control_value to PWM values
    int pwmValueCW = (1.0 - control_value) * 256;  // Inverse mapping for Cool White
    int pwmValueWW = control_value * 256;          // Direct mapping for Warm White

    // Set PWM duty cycles using updated ledcWrite function
    ledcWrite(pinCW, pwmValueCW);
    ledcWrite(pinWW, pwmValueWW);

    // Delay to allow sensors to integrate light
    delay(40); // Delay matching the integration time

    // Get sensor data from VCNL4040 (U1)
    uint16_t proximity = vcnl4040.getProximity();
    uint16_t ambient = vcnl4040.getLux();

    // Get sensor data from VEML6040 (LS-B) on I2C_0
    uint16_t red_LSB = veml6040_LSB.getRed();
    uint16_t green_LSB = veml6040_LSB.getGreen();
    uint16_t blue_LSB = veml6040_LSB.getBlue();
    uint16_t white_LSB = veml6040_LSB.getWhite();
    uint16_t cct_LSB = veml6040_LSB.getCCT();
    float lux_LSB = veml6040_LSB.getAmbientLight();

    // Get sensor data from VEML6040 (LS-A) on I2C_1
    uint16_t red_LSA = veml6040_LSA.getRed();
    uint16_t green_LSA = veml6040_LSA.getGreen();
    uint16_t blue_LSA = veml6040_LSA.getBlue();
    uint16_t white_LSA = veml6040_LSA.getWhite();
    uint16_t cct_LSA = veml6040_LSA.getCCT();
    float lux_LSA = veml6040_LSA.getAmbientLight();

    // Print the control value and sensor data
    Serial.print(control_value, 3);  // Print control value with 3 decimal places
    Serial.print(",");
    Serial.print(proximity);
    Serial.print(",");
    Serial.print(ambient);
    Serial.print(",");
    Serial.print(red_LSB);
    Serial.print(",");
    Serial.print(green_LSB);
    Serial.print(",");
    Serial.print(blue_LSB);
    Serial.print(",");
    Serial.print(white_LSB);
    Serial.print(",");
    Serial.print(cct_LSB);
    Serial.print(",");
    Serial.print(lux_LSB, 2);
    Serial.print(",");
    Serial.print(red_LSA);
    Serial.print(",");
    Serial.print(green_LSA);
    Serial.print(",");
    Serial.print(blue_LSA);
    Serial.print(",");
    Serial.print(white_LSA);
    Serial.print(",");
    Serial.print(cct_LSA);
    Serial.print(",");
    Serial.println(lux_LSA, 2);

    // Small delay to avoid overwhelming the serial output
    delay(10);
  }

  // Decrease control_value from 1.000 back to 0.000
  for (float control_value = 1.000; control_value >= 0.000; control_value -= 0.001) {
    // Map control_value to PWM values
    int pwmValueCW = (1.0 - control_value) * 255;  // Inverse mapping for Cool White
    int pwmValueWW = control_value * 255;          // Direct mapping for Warm White

    // Set PWM duty cycles using updated ledcWrite function
    ledcWrite(pinCW, pwmValueCW);
    ledcWrite(pinWW, pwmValueWW);

    // Delay to allow sensors to integrate light
    delay(40); // Delay matching the integration time

    // Get sensor data from VCNL4040 (U1)
    uint16_t proximity = vcnl4040.getProximity();
    uint16_t ambient = vcnl4040.getLux();

    // Get sensor data from VEML6040 (LS-B) on I2C_0
    uint16_t red_LSB = veml6040_LSB.getRed();
    uint16_t green_LSB = veml6040_LSB.getGreen();
    uint16_t blue_LSB = veml6040_LSB.getBlue();
    uint16_t white_LSB = veml6040_LSB.getWhite();
    uint16_t cct_LSB = veml6040_LSB.getCCT();
    float lux_LSB = veml6040_LSB.getAmbientLight();

    // Get sensor data from VEML6040 (LS-A) on I2C_1
    uint16_t red_LSA = veml6040_LSA.getRed();
    uint16_t green_LSA = veml6040_LSA.getGreen();
    uint16_t blue_LSA = veml6040_LSA.getBlue();
    uint16_t white_LSA = veml6040_LSA.getWhite();
    uint16_t cct_LSA = veml6040_LSA.getCCT();
    float lux_LSA = veml6040_LSA.getAmbientLight();

    // Print the control value and sensor data
    Serial.print(control_value, 3);  // Print control value with 3 decimal places
    Serial.print(",");
    Serial.print(proximity);
    Serial.print(",");
    Serial.print(ambient);
    Serial.print(",");
    Serial.print(red_LSB);
    Serial.print(",");
    Serial.print(green_LSB);
    Serial.print(",");
    Serial.print(blue_LSB);
    Serial.print(",");
    Serial.print(white_LSB);
    Serial.print(",");
    Serial.print(cct_LSB);
    Serial.print(",");
    Serial.print(lux_LSB, 2);
    Serial.print(",");
    Serial.print(red_LSA);
    Serial.print(",");
    Serial.print(green_LSA);
    Serial.print(",");
    Serial.print(blue_LSA);
    Serial.print(",");
    Serial.print(white_LSA);
    Serial.print(",");
    Serial.print(cct_LSA);
    Serial.print(",");
    Serial.println(lux_LSA, 2);

    // Small delay to avoid overwhelming the serial output
    delay(10);
  }
}
