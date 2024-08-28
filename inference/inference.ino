#include <Adafruit_APDS9960.h>
#include <driver/ledc.h>
#include <TensorFlowLite.h>
#include <tensorflow/lite/experimental/micro/kernels/activations.h>
#include <tensorflow/lite/experimental/micro/kernels/fully_connected.h>
#include <tensorflow/lite/experimental/micro/kernels/memory.h>
#include <tensorflow/lite/experimental/micro/kernels/quantize.h>
#include <tensorflow/lite/experimental/micro/kernels/transpose.h>
#include <tensorflow/lite/experimental/micro/kernels/mean.h>
#include <tensorflow/lite/experimental/micro/kernels/multi_head_attention.h>
#include <tensorflow/lite/experimental/micro/kernels/unary.h>
#include <tensorflow/lite/experimental/micro/kernels/batch_norm.h>
#include <tensorflow/lite/experimental/micro/kernels/concat.h>
#include <tensorflow/lite/experimental/micro/kernels/expand_dims.h>
#include <tensorflow/lite/experimental/micro/kernels/split.h>
#include <tensorflow/lite/experimental/micro/kernels/softmax.h>

// Define the TensorFlow Lite model
extern "C" const uint8_t model_tflite_start[] asm("_binary_model_tflite_start");
extern "C" const uint8_t model_tflite_end[] asm("_binary_model_tflite_end");

const int pwmFreq = 1000;
const int pwmResolution = 8;
const int pwmChannelCW = 0;
const int pwmChannelWW = 1;

const int pinCW = 32;
const int pinWW = 35;

Adafruit_APDS9960 apds;
tflite::MicroInterpreter* interpreter;
tflite::Model* model;
tflite::MicroMutableOpResolver<10> micro_op_resolver;
tflite::MicroErrorReporter micro_error_reporter;
tflite::MicroInterpreter* interpreter;
TfLiteTensor* input_tensor;
TfLiteTensor* output_tensor;

void setup() {
  Serial.begin(115200);

  // Initialize the APDS9960 sensor
  if (!apds.begin()) {
    Serial.println("Failed to initialize device! Please check your wiring.");
    while (1);
  }
  Serial.println("Device initialized!");

  // Set up PWM for Cool White and Warm White LEDs
  ledcSetup(pwmChannelCW, pwmFreq, pwmResolution);
  ledcSetup(pwmChannelWW, pwmFreq, pwmResolution);
  ledcAttachPin(pinCW, pwmChannelCW);
  ledcAttachPin(pinWW, pwmChannelWW);

  // Initialize TensorFlow Lite
  model = tflite::GetModel(model_tflite_start);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("Model version is not supported.");
    while (1);
  }

  // Set up the interpreter
  micro_op_resolver.AddFullyConnected();
  micro_op_resolver.AddSoftmax();
  // Add other operators as needed

  static tflite::MicroInterpreter static_interpreter(model, micro_op_resolver, tensor_arena, kTensorArenaSize, &micro_error_reporter);
  interpreter = &static_interpreter;

  interpreter->AllocateTensors();
  input_tensor = interpreter->input(0);
  output_tensor = interpreter->output(0);
}

void loop() {
  // Wait for color data to be ready
  while (!apds.colorDataReady()) {
    delay(5);
  }

  // Get the color data from the sensor
  uint16_t r, g, b, c;
  apds.getColorData(&r, &g, &b, &c);

  // Normalize the color data (example normalization)
  float input_data[4] = { (float)r / 65535.0, (float)g / 65535.0, (float)b / 65535.0, (float)c / 65535.0 };

  // Set the input tensor
  memcpy(input_tensor->data.f, input_data, sizeof(input_data));

  // Run inference
  if (interpreter->Invoke() != kTfLiteOk) {
    Serial.println("Failed to invoke the model.");
    return;
  }

  // Get the output tensor
  float control_value = output_tensor->data.f[0];

  // Map control_value to PWM values
  int pwmValueCW = (1.0 - control_value) * 255;  // Inverse mapping for Cool White
  int pwmValueWW = control_value * 255;          // Direct mapping for Warm White

  // Set PWM duty cycles
  ledcWrite(pwmChannelCW, pwmValueCW);
  ledcWrite(pwmChannelWW, pwmValueWW);

  // Print the results
  Serial.print("Control Value: ");
  Serial.print(control_value);
  Serial.print(" PWM CW: ");
  Serial.print(pwmValueCW);
  Serial.print(" PWM WW: ");
  Serial.println(pwmValueWW);

  delay(500);  // Adjust delay as needed
}
