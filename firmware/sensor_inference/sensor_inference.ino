#include <veml6040_sunbox.h>
#include <Wire.h>
#include <Adafruit_VCNL4040.h>
#include <tflm_esp32.h>
#include <eloquent_tinyml.h>
#include <driver/ledc.h>

// Include your trained TensorFlow Lite model header file
#include "tf_model.h"  // Replace with your actual model header file

// Define the arena size for TensorFlow Lite Micro
#define ARENA_SIZE 4096  // Adjust based on your model's requirements

// Initialize the EloquentTinyML model
Eloquent::TF::Sequential<TF_NUM_OPS, ARENA_SIZE> tf;

// Define PWM settings
const int pwmFreq = 1000;    // Frequency of PWM
const int pwmResolution = 8; // 8-bit resolution (0-255)

// Define PWM pins
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

// Define the number of input features
#define NUM_FEATURES 14

// Min and Max values from the MinMaxScaler
// Replace these values with the actual min and max values from your scaler
float data_min[NUM_FEATURES] = {
122,    431,   1246,    920,    380,   2007,   2687,    231.55,  895,
  640,    269,   1651,   2546,    161.08,
};

float data_max[NUM_FEATURES] = {
152,    4100,   14696,   13907,    8277,   20945,    4889,    3500.11,
 11081,   10318,    5879,   18071,    4591,    2596.83,
};


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



  // Initialize the TensorFlow Lite model
  Serial.println("Initializing TensorFlow Lite model...");
  registerNetworkOps(tf);
  
  delay(3000);
  Serial.println("__TENSORFLOW ESP32 RGBW CONTROL__");
  
  // Configure input/output sizes
  tf.setNumInputs(TF_NUM_INPUTS);
  tf.setNumOutputs(TF_NUM_OUTPUTS);

  // Add required operations
  tf.resolver.AddFullyConnected();

  // Begin the model
  while (!tf.begin(tfModel).isOk()) {
    Serial.println(tf.exception.toString());
    delay(1000);  // Retry every second
    Serial.println("TensorFlow Lite model initialization failed!");
  }
  Serial.println("TensorFlow Lite model initialized!");
}

void loop() {
  // Delay to allow sensors to integrate light
  delay(160); // Delay matching the integration time

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

  // Prepare input for the model
  float input[NUM_FEATURES] = {
    (float)proximity,
    (float)ambient,
    (float)red_LSB,
    (float)green_LSB,
    (float)blue_LSB,
    (float)white_LSB,
    (float)cct_LSB,
    lux_LSB,
    (float)red_LSA,
    (float)green_LSA,
    (float)blue_LSA,
    (float)white_LSA,
    (float)cct_LSA,
    lux_LSA
  };

  // Apply MinMax Scaling
  for (int i = 0; i < NUM_FEATURES; i++) {
    input[i] = (input[i] - data_min[i]) / (data_max[i] - data_min[i]);
    // Ensure the scaled value is within [0, 1]
    if (input[i] < 0.0) input[i] = 0.0;
    if (input[i] > 1.0) input[i] = 1.0;
  }

  // Run inference
  if (!tf.predict(input).isOk()) {
    Serial.println(tf.exception.toString());
    return;
  }

  // Retrieve the predicted control value
  float control_value = tf.output(0);

  // Ensure control_value is within [0.0, 1.0]
  if (control_value < 0.0) control_value = 0.0;
  if (control_value > 1.0) control_value = 1.0;

  // Map control_value to PWM values
  int pwmValueCW = (1.0 - control_value) * 255;  // Inverse mapping for Cool White
  int pwmValueWW = control_value * 255;          // Direct mapping for Warm White

  // Set PWM duty cycles
  ledcWrite(pinCW, pwmValueCW);
  ledcWrite(pinWW, pwmValueWW);

  // Print the sensor data and control value
  Serial.println("Sensor Data:");
  Serial.printf("Proximity: %d, Ambient: %d\n", proximity, ambient);
  Serial.printf("LS-B - R: %d, G: %d, B: %d, W: %d, CCT: %d, Lux: %.2f\n", red_LSB, green_LSB, blue_LSB, white_LSB, cct_LSB, lux_LSB);
  Serial.printf("LS-A - R: %d, G: %d, B: %d, W: %d, CCT: %d, Lux: %.2f\n", red_LSA, green_LSA, blue_LSA, white_LSA, cct_LSA, lux_LSA);
  Serial.printf("Control Value: %.3f\n", control_value);

  // Small delay to avoid overwhelming the serial output
  delay(20);
}
