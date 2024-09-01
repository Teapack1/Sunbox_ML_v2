#include <EloquentTinyML.h>
#include <Adafruit_APDS9960.h>
#include <driver/ledc.h>
// sine_model.h contains the array you exported from the previous step with xxd or tinymlgen
#include "LED_200.h"

#define NUMBER_OF_INPUTS 4
#define NUMBER_OF_OUTPUTS 1
// in future projects you may need to tweek this value: it's a trial and error process
#define TENSOR_ARENA_SIZE 4096

Eloquent::TinyML::TfLite<NUMBER_OF_INPUTS, NUMBER_OF_OUTPUTS, TENSOR_ARENA_SIZE> ml;
Adafruit_APDS9960 apds;

// Define PWM settings
const int pwmFreq = 1000;  // Frequency of PWM
const int pwmResolution = 8;  // 8-bit resolution (0-255)
const int pwmChannelCW = 0;  // Channel for Cool White LED
const int pwmChannelWW = 1;  // Channel for Warm White LED

const int pinCW = 32;  // GPIO for Cool White LED
const int pinWW = 35;  // GPIO for Warm White LED

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("__TENSORFLOW ESP32 RGBW CONTROL__");

        // Initialize the APDS9960 sensor
    if (!apds.begin()) {
       Serial.println("Failed to initialize device! Please check your wiring.");
        while (1);
    }
    Serial.println("Device initialized!");
    // Enable color sensing
    apds.enableColor(true);

    // Set up PWM for Cool White and Warm White LEDs
    ledcSetup(pwmChannelCW, pwmFreq, pwmResolution);
    ledcSetup(pwmChannelWW, pwmFreq, pwmResolution);

    ledcAttachPin(pinCW, pwmChannelCW);
    ledcAttachPin(pinWW, pwmChannelWW);

    // Add required ops (update this according to your model's needs)

    ml.begin(modelData);
}

void loop() {

        // Wait for sensor data to be ready
    while (!apds.colorDataReady()) {
        delay(5);
    }
        // Get the color data from the sensor
    uint16_t r, g, b, w;
    apds.getColorData(&r, &g, &b, &w);

    // pick up a random x and predict its sine
    //float r = 20 * (float)random(100) / 100;
    //float g = 20 * (float)random(100) / 100;
    //float b = 20 * (float)random(100) / 100;
    //float w = 50 * (float)random(100) / 100;
    float input[4] = { (float)r, (float)g, (float)b, (float)w };
    float control_value = ml.predict(input);

        // Map control_value to PWM values
    int pwmValueCW = (1.0 - control_value) * 255;  // Inverse mapping for Cool White
    int pwmValueWW = control_value * 255;          // Direct mapping for Warm White

    // Set PWM duty cycles
    ledcWrite(pwmChannelCW, pwmValueCW);
    ledcWrite(pwmChannelWW, pwmValueWW);

    // Print the sensor data and control value
    Serial.printf("R: %d, G: %d, B: %d, W: %d -> Control Value: %.3f\n", r, g, b, w, control_value);

    // Small delay to avoid overwhelming the serial output
    delay(1000);
}