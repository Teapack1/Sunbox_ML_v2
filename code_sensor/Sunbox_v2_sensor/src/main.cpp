#include <Arduino.h>
#include <Wire.h>
#include <TensorFlowLite_ESP32.h>
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>

// Include your model header file here
// #include "model.h"

// I2C addresses and pins
#define SENSOR1_ADDR 0x29
#define SENSOR2_ADDR 0x39
#define SENSOR3_ADDR 0x39
#define DISPLAY_ADDR 0x3C
#define WARM_WHITE_PIN 25
#define COLD_WHITE_PIN 26
#define TOUCH_PIN 4

// PWM properties
#define FREQ 5000
#define WARM_WHITE_CHANNEL 0
#define COLD_WHITE_CHANNEL 1
#define RESOLUTION 8

// Display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// EEPROM settings
#define EEPROM_SIZE 1024
#define MAX_SAMPLES 100

// TensorFlow Lite globals
tflite::MicroErrorReporter tflErrorReporter;
tflite::AllOpsResolver tflOpsResolver;
const tflite::Model* tflModel = nullptr;
tflite::MicroInterpreter* tflInterpreter = nullptr;
TfLiteTensor* tflInputTensor = nullptr;
TfLiteTensor* tflOutputTensor = nullptr;

// Create a buffer for TensorFlow Lite arena
constexpr int kTensorArenaSize = 8 * 1024;
uint8_t tensorArena[kTensorArenaSize];

// Create display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Global variables
enum Mode { DATA_COLLECTION, INFERENCE };
Mode currentMode = INFERENCE;
int sampleCount = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  EEPROM.begin(EEPROM_SIZE);

  // Set up PWM for LEDs
  ledcSetup(WARM_WHITE_CHANNEL, FREQ, RESOLUTION);
  ledcSetup(COLD_WHITE_CHANNEL, FREQ, RESOLUTION);
  ledcAttachPin(WARM_WHITE_PIN, WARM_WHITE_CHANNEL);
  ledcAttachPin(COLD_WHITE_PIN, COLD_WHITE_CHANNEL);

  // Set up touch button
  touchAttachInterrupt(TOUCH_PIN, changeModeCallback, 40);

  // Set up display
  if(!display.begin(SSD1306_SWITCHCAPVCC, DISPLAY_ADDR)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Starting up...");
  display.display();

  // Set up TensorFlow Lite model
  tflModel = tflite::GetModel(g_model);
  if (tflModel->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("Model schema mismatch!");
    return;
  }

  // Create interpreter
  tflInterpreter = new tflite::MicroInterpreter(tflModel, tflOpsResolver, tensorArena, kTensorArenaSize, &tflErrorReporter);

  // Allocate tensors
  if (tflInterpreter->AllocateTensors() != kTfLiteOk) {
    Serial.println("Failed to allocate tensors!");
    return;
  }

  // Get pointers to input and output tensors
  tflInputTensor = tflInterpreter->input(0);
  tflOutputTensor = tflInterpreter->output(0);

  // Read sample count from EEPROM
  EEPROM.get(0, sampleCount);
}

void loop() {
  float sensor1[4], sensor2[4], sensor3[4];
  readRGBWSensor(SENSOR1_ADDR, sensor1);
  readRGBWSensor(SENSOR2_ADDR, sensor2);
  readRGBWSensor(SENSOR3_ADDR, sensor3);

  if (currentMode == DATA_COLLECTION) {
    collectData(sensor1, sensor2, sensor3);
  } else {
    runInference(sensor1, sensor2, sensor3);
  }

  updateDisplay();
  delay(100);
}

void changeModeCallback() {
  currentMode = (currentMode == DATA_COLLECTION) ? INFERENCE : DATA_COLLECTION;
}

void collectData(float* sensor1, float* sensor2, float* sensor3) {
  if (sampleCount >= MAX_SAMPLES) {
    Serial.println("Maximum samples reached");
    return;
  }

  // Collect RGBW data
  float data[12];
  for (int i = 0; i < 4; i++) {
    data[i] = sensor1[i];
    data[i+4] = sensor2[i];
    data[i+8] = sensor3[i];
  }

  // Get user input for warm and cold white values
  Serial.println("Enter warm white value (0-255):");
  while (!Serial.available()) {}
  int warmWhite = Serial.parseInt();
  
  Serial.println("Enter cold white value (0-255):");
  while (!Serial.available()) {}
  int coldWhite = Serial.parseInt();

  // Store data in EEPROM
  int addr = sizeof(int) + sampleCount * (12 * sizeof(float) + 2 * sizeof(int));
  EEPROM.put(addr, data);
  addr += 12 * sizeof(float);
  EEPROM.put(addr, warmWhite);
  addr += sizeof(int);
  EEPROM.put(addr, coldWhite);

  sampleCount++;
  EEPROM.put(0, sampleCount);
  EEPROM.commit();

  Serial.printf("Sample %d collected\n", sampleCount);
}

void runInference(float* sensor1, float* sensor2, float* sensor3) {
  // Prepare input tensor
  for (int i = 0; i < 4; i++) {
    tflInputTensor->data.f[i] = sensor1[i];
    tflInputTensor->data.f[i+4] = sensor2[i];
    tflInputTensor->data.f[i+8] = sensor3[i];
  }

  // Run inference
  if (tflInterpreter->Invoke() != kTfLiteOk) {
    Serial.println("Invoke failed!");
    return;
  }

  // Read output tensor
  int warmWhiteDuty = (int)(tflOutputTensor->data.f[0] * 255);
  int coldWhiteDuty = (int)(tflOutputTensor->data.f[1] * 255);

  // Set LED duty cycles
  ledcWrite(WARM_WHITE_CHANNEL, warmWhiteDuty);
  ledcWrite(COLD_WHITE_CHANNEL, coldWhiteDuty);

  Serial.printf("Warm White: %d, Cold White: %d\n", warmWhiteDuty, coldWhiteDuty);
}

void readRGBWSensor(int address, float* rgbw) {
  Wire.beginTransmission(address);
  Wire.write(0x00); // Command to read RGBW values
  Wire.endTransmission();

  Wire.requestFrom(address, 8);
  if (Wire.available() == 8) {
    for (int i = 0; i < 4; i++) {
      uint16_t value = Wire.read() | (Wire.read() << 8);
      rgbw[i] = value / 65535.0; // Normalize to 0-1 range
    }
  }
}

void updateDisplay() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.printf("Mode: %s\n", (currentMode == DATA_COLLECTION) ? "Collect" : "Inference");
  display.printf("Samples: %d\n", sampleCount);
  // Add more information as needed
  display.display();
}