#include <Adafruit_APDS9960.h>
#include <driver/ledc.h>

// Create an instance of the sensor
Adafruit_APDS9960 apds;

// Define PWM settings
const int pwmFreq = 1000;  // Frequency of PWM
const int pwmResolution = 8;  // 8-bit resolution (0-255)
const int pwmChannelCW = 0;  // Channel for Cool White LED
const int pwmChannelWW = 1;  // Channel for Warm White LED

const int pinCW = 32;  // GPIO for Cool White LED
const int pinWW = 25;  // GPIO for Warm White LED

void setup() {
  Serial.begin(115200);

  // Initialize the APDS9960 sensor
  if (!apds.begin()) {
    Serial.println("Failed to initialize device! Please check your wiring.");
    while (1);
  }
  Serial.println("Device initialized!");

  // Enable color sensing
  apds.enableColor(true);

  // Set up PWM for Cool White and Warm White LEDs
  ledcAttach(pinCW, pwmFreq, pwmResolution);
  ledcAttach(pinWW, pwmFreq, pwmResolution);
}

void loop() {
  // Increase control_value from 0.000 to 1.000
  for (float control_value = 0.000; control_value <= 1.000; control_value += 0.001) {
    // Map control_value to PWM values
    int pwmValueCW = (1.0 - control_value) * 255;  // Inverse mapping for Cool White
    int pwmValueWW = control_value * 255;          // Direct mapping for Warm White

    // Set PWM duty cycles
    ledcWrite(pinCW, pwmValueCW);
    ledcWrite(pinWW, pwmValueWW);

    // Wait for sensor data to be ready
    while (!apds.colorDataReady()) {
      delay(1);
    }

    // Get the color data from the sensor
    uint16_t r, g, b, c;
    apds.getColorData(&r, &g, &b, &c);

    // Print the control value and sensor data
    Serial.print(control_value, 3);  // Print control value with 3 decimal places
    Serial.print(",");
    Serial.print(r);
    Serial.print(",");
    Serial.print(g);
    Serial.print(",");
    Serial.print(b);
    Serial.print(",");
    Serial.println(c);

    // Small delay to avoid overwhelming the serial output
    delay(1);
  }

  // Decrease control_value from 1.000 back to 0.000
  for (float control_value = 1.000; control_value >= 0.000; control_value -= 0.001) {
    // Map control_value to PWM values
    int pwmValueCW = (1.0 - control_value) * 255;  // Inverse mapping for Cool White
    int pwmValueWW = control_value * 255;          // Direct mapping for Warm White

    // Set PWM duty cycles
    ledcWrite(pinCW, pwmValueCW);
    ledcWrite(pinWW, pwmValueWW);

    // Wait for sensor data to be ready
    while (!apds.colorDataReady()) {
      delay(1);
    }

    // Get the color data from the sensor
    uint16_t r, g, b, c;
    apds.getColorData(&r, &g, &b, &c);

    // Print the control value and sensor data
    Serial.print(control_value, 3);  // Print control value with 3 decimal places
    Serial.print(",");
    Serial.print(r);
    Serial.print(",");
    Serial.print(g);
    Serial.print(",");
    Serial.print(b);
    Serial.print(",");
    Serial.println(c);

    // Small delay to avoid overwhelming the serial output
    delay(1);
  }
}
