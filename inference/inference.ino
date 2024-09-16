/**
 * Run a TensorFlow model to predict the IRIS dataset
 * For a complete guide, visit
 * https://eloquentarduino.com/tensorflow-lite-esp32
 */
#include <driver/ledc.h>
#include "LED_200.h"
#include <tflm_esp32.h>
#include <eloquent_tinyml.h>
#include <Adafruit_APDS9960.h>

// this is trial-and-error process
// when developing a new model, start with a high value
// (e.g. 10000), then decrease until the model stops
// working as expected
#define ARENA_SIZE 4096

Eloquent::TF::Sequential<TF_NUM_OPS, ARENA_SIZE> tf;
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
    registerNetworkOps(tf);
    
    delay(3000);
    Serial.println("__TENSORFLOW ESP32 RGBW CONTROL__");

    // Initialize the APDS9960 sensor
    if (!apds.begin()) {
        Serial.println("Failed to initialize device! Please check your wiring.");
        while (1);
    }
    Serial.println("Device initialized!");    
    // Enable color sensing
    apds.enableColor(true);
    
    // Set up PWM for Cool White and Warm White LEDs using the new `ledcAttach` function
    ledcAttach(pinCW, pwmFreq, pwmResolution);
    ledcAttach(pinWW, pwmFreq, pwmResolution);


    // configure input/output
    // (not mandatory if you generated the .h model
    // using the everywhereml Python package)
    tf.setNumInputs(TF_NUM_INPUTS);
    tf.setNumOutputs(TF_NUM_OUTPUTS);
    // add required ops
    // (not mandatory if you generated the .h model
    // using the everywhereml Python package)
    tf.resolver.AddFullyConnected();
  
    while (!tf.begin(tfModel).isOk()) 
        Serial.println(tf.exception.toString());
        delay(1000);  // Retry every second
}


void loop() {

        // Wait for sensor data to be ready
    while (!apds.colorDataReady()) {
        delay(5);
    }

    // Get the color data from the sensor
    uint16_t r, g, b, w;
    apds.getColorData(&r, &g, &b, &w);

    
    // Prepare input for the model
    float input[4] = { (float)r, (float)g, (float)b, (float)w };

    // classify class 0
    if (!tf.predict(input).isOk()) {
        Serial.println(tf.exception.toString());
        return;
    }

        // Retrieve the predicted control value
    float control_value = tf.output(0);

    
    // Map control_value to PWM values
    int pwmValueCW = (1.0 - control_value) * 255;  // Inverse mapping for Cool White
    int pwmValueWW = control_value * 255;          // Direct mapping for Warm White

    // Set PWM duty cycles
    ledcWrite(pinCW, pwmValueCW);
    ledcWrite(pinWW, pwmValueWW);
    
    // Print the sensor data and control value
    Serial.printf("R: %d, G: %d, B: %d, W: %d -> Control Value: %.3f\n", r, g, b, w, control_value);

    // Small delay to avoid overwhelming the serial output
    delay(20);

    // how long does it take to run a single prediction?
    Serial.print("It takes ");
    Serial.print(tf.benchmark.microseconds());
    Serial.println("us for a single prediction");
    
    delay(20);
}
