#include <PID_v1.h>
#include <Arduino.h>

// Forward declarations
double getEncoderSpeed(int motorIndex);
void setMotorSpeed(int motorIndex, double pwmValue);
void movementTask(void *parameter);
void pidControlTask(void *parameter);

// ----- Motor and Encoder Configuration -----
// Each motor uses two PWM channels (one for forward and one for reverse) and two encoder inputs.
struct MotorConfig {
  uint8_t pwmA;  // Forward PWM channel
  uint8_t pwmB;  // Reverse PWM channel
  uint8_t encA;  // Encoder channel A
  uint8_t encB;  // Encoder channel B
};

// Board pin assignments as provided:
// Motor M1: PWM_A=GPIO4, PWM_B=GPIO5, Enc_A=GPIO6, Enc_B=GPIO7
// Motor M2: PWM_A=GPIO15, PWM_B=GPIO16, Enc_A=GPIO47, Enc_B=GPIO48
// Motor M3: PWM_A=GPIO9, PWM_B=GPIO10, Enc_A=GPIO11, Enc_B=GPIO12
// Motor M4: PWM_A=GPIO13, PWM_B=GPIO14, Enc_A=GPIO1, Enc_B=GPIO2
MotorConfig motors[4] = {
  { 5, 4, 6, 7  },   // Motor 0: M1
  { 15, 16, 47, 48 }, // Motor 1: M2
  { 9, 10, 11, 12 },  // Motor 2: M3
  { 14, 13, 1, 2 }    // Motor 3: M4
};

// ----- PID Variables -----
// For each motor: target speed (setpoint), measured speed (input), and PID output (PWM value)
double setpoints[4] = { 100, 100, 100, 100 }; // Initial desired speeds (tune as needed)
double inputs[4]    = { 0, 0, 0, 0 };          // Measured speeds from encoders
double outputs[4]   = { 0, 0, 0, 0 };          // PID controller outputs

// PID tuning parameters (adjust these for your system)
double Kp = 2.0, Ki = 5.0, Kd = 1.0;

// Create PID controllers for each motor
PID* pids[4];

void setup() {
  // Use Serial0 for ESP32-S3 (often Serial0 is the default Serial port)
  Serial0.begin(115200);

  // Initialize motor PWM and encoder pins
  for (int i = 0; i < 4; i++) {
    pinMode(motors[i].pwmA, OUTPUT);
    pinMode(motors[i].pwmB, OUTPUT);
    pinMode(motors[i].encA, INPUT);
    pinMode(motors[i].encB, INPUT);
  }

  // Initialize PID controllers for each motor
  for (int i = 0; i < 4; i++) {
    pids[i] = new PID(&inputs[i], &outputs[i], &setpoints[i], Kp, Ki, Kd, DIRECT);
    pids[i]->SetMode(AUTOMATIC);
    // Set output limits to match PWM range (-255 to 255)
    pids[i]->SetOutputLimits(-255, 255);
  }

  // Create a FreeRTOS task for the PID control loop.
  xTaskCreate(
    pidControlTask,      // Task function
    "PIDControlTask",    // Task name
    4096,                // Stack size (in words)
    NULL,                // Parameter to pass
    1,                   // Priority
    NULL                 // Task handle
  );

  // Create a FreeRTOS task for the movement pattern.
  xTaskCreate(
    movementTask,        // Task function
    "MovementTask",      // Task name
    2048,                // Stack size (in words)
    NULL,                // Parameter to pass
    1,                   // Priority
    NULL                 // Task handle
  );
}

// loop() is not used when employing FreeRTOS tasks.
void loop() {
  // Do nothing here.
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

// ----- PID Control Task -----
// This task continuously reads encoder speeds, computes PID, and updates motor outputs.
void pidControlTask(void *parameter) {
  for (;;) {
    for (int i = 0; i < 4; i++) {
      inputs[i] = getEncoderSpeed(i);  // Replace with actual encoder reading
      pids[i]->Compute();
      setMotorSpeed(i, outputs[i]);
    }

    // Debug print on Serial0
    Serial0.print("Inputs: ");
    for (int i = 0; i < 4; i++) {
      Serial0.print(inputs[i]);
      Serial0.print("  ");
    }
    Serial0.print(" Setpoints: ");
    for (int i = 0; i < 4; i++) {
      Serial0.print(setpoints[i]);
      Serial0.print("  ");
    }
    Serial0.print(" Outputs: ");
    for (int i = 0; i < 4; i++) {
      Serial0.print(outputs[i]);
      Serial0.print("  ");
    }
    Serial0.println();

    vTaskDelay(100 / portTICK_PERIOD_MS);  // PID loop delay
  }
}

// ----- Movement Pattern Task -----
// This task changes the setpoints: forward for 1 second, stop for 1 second, then backward for 1 second.
void movementTask(void *parameter) {
  for (;;) {
    // Move forward: positive speed
    for (int i = 0; i < 4; i++) {
      setpoints[i] = -100;
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // Stop: zero speed
    // for (int i = 0; i < 4; i++) {
    //   setpoints[i] = 0;
    // }
    // vTaskDelay(1000 / portTICK_PERIOD_MS);

    // // Move backward: negative speed
    // for (int i = 0; i < 4; i++) {
    //   setpoints[i] = -100;
    // }
    // vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// ----- Dummy Encoder Reading Function -----
// Replace this with your actual encoder reading logic.
double getEncoderSpeed(int motorIndex) {
  // For demonstration, return a dummy constant value.
  return 80.0;
}

// ----- Function to Set Motor Speed via PWM -----
// This function sets motor speed and direction based on the sign of pwmValue.
// For forward motion, pwmA is used; for reverse, pwmB is used.
void setMotorSpeed(int motorIndex, double pwmValue) {
  if (pwmValue >= 0) {
    analogWrite(motors[motorIndex].pwmA, (int)pwmValue);
    analogWrite(motors[motorIndex].pwmB, 0);
  } else {
    analogWrite(motors[motorIndex].pwmA, 0);
    analogWrite(motors[motorIndex].pwmB, (int)(-pwmValue));
  }
}
