#include <PID_v1.h>
#include <Arduino.h>
#include <ESP32Encoder.h>

//---------------------------------------------------------------------
// Global Encoder Instances and Variables
//---------------------------------------------------------------------
ESP32Encoder encoder0;
ESP32Encoder encoder1;
ESP32Encoder encoder2;
ESP32Encoder encoder3;

// Array to store previous encoder counts (one per motor)
long prevCounts[4] = {0, 0, 0, 0};

//---------------------------------------------------------------------
// Forward Declarations
//---------------------------------------------------------------------
double getEncoderSpeed(int motorIndex);
void setMotorSpeed(int motorIndex, double pwmValue);
void movementTask(void *parameter);
void pidControlTask(void *parameter);

//---------------------------------------------------------------------
// Motor and Encoder Configuration
//---------------------------------------------------------------------
struct MotorConfig {
  uint8_t pwmA;  // Forward PWM channel
  uint8_t pwmB;  // Reverse PWM channel
  uint8_t encA;  // Encoder channel A
  uint8_t encB;  // Encoder channel B
};

// Board pin assignments (as provided):
// M1: PWM_A = GPIO4,  PWM_B = GPIO5,  Enc_A = GPIO6,  Enc_B = GPIO7
// M2: PWM_A = GPIO15, PWM_B = GPIO16, Enc_A = GPIO47, Enc_B = GPIO48
// M3: PWM_A = GPIO9,  PWM_B = GPIO10, Enc_A = GPIO11, Enc_B = GPIO12
// M4: PWM_A = GPIO13, PWM_B = GPIO14, Enc_A = GPIO1,  Enc_B = GPIO2
// (Note: The following array uses the order provided in your sketch.)
MotorConfig motors[4] = {
  { 5, 4, 6, 7  },    // Motor 0: M1
  { 15, 16, 47, 48 },  // Motor 1: M2
  { 9, 10, 11, 12 },   // Motor 2: M3
  { 13, 14, 1, 2 }     // Motor 3: M4
};

//---------------------------------------------------------------------
// PID Variables
//---------------------------------------------------------------------
double setpoints[4] = { 100, 100, 100, 100 }; // Desired speeds (counts/sec)
double inputs[4]    = { 0, 0, 0, 0 };          // Measured speeds (counts/sec)
double outputs[4]   = { 0, 0, 0, 0 };          // PID outputs (PWM value)

double Kp = 2.0, Ki = 5.0, Kd = 1.0;           // Tuning parameters

// Array of PID controller pointers, one per motor
PID* pids[4];

//---------------------------------------------------------------------
// Setup Function
//---------------------------------------------------------------------
void setup() {
  // Use Serial0 for ESP32-S3 debugging
  Serial0.begin(115200);

  // Initialize motor PWM and encoder pins
  for (int i = 0; i < 4; i++) {
    pinMode(motors[i].pwmA, OUTPUT);
    pinMode(motors[i].pwmB, OUTPUT);
    pinMode(motors[i].encA, INPUT);
    pinMode(motors[i].encB, INPUT);
  }

  // Attach encoders in half-quadrature mode
  // (Ensure your wiring and the encoder library settings match your hardware)
  encoder0.attachHalfQuad(motors[0].encA, motors[0].encB);
  encoder1.attachHalfQuad(motors[1].encA, motors[1].encB);
  encoder2.attachHalfQuad(motors[2].encA, motors[2].encB);
  encoder3.attachHalfQuad(motors[3].encA, motors[3].encB);

  // Clear initial encoder counts
  encoder0.clearCount();
  encoder1.clearCount();
  encoder2.clearCount();
  encoder3.clearCount();

  // Initialize PID controllers for each motor
  for (int i = 0; i < 4; i++) {
    pids[i] = new PID(&inputs[i], &outputs[i], &setpoints[i], Kp, Ki, Kd, DIRECT);
    pids[i]->SetMode(AUTOMATIC);
    // Set PID output limits to match the PWM range (-255 to 255)
    pids[i]->SetOutputLimits(-255, 255);
  }

  // Create FreeRTOS tasks for PID control and movement pattern
  xTaskCreate(pidControlTask, "PIDControlTask", 4096, NULL, 1, NULL);
  xTaskCreate(movementTask, "MovementTask", 2048, NULL, 1, NULL);
}

//---------------------------------------------------------------------
// loop() Function (Unused)
//---------------------------------------------------------------------
void loop() {
  // With FreeRTOS tasks, loop() is not used.
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

//---------------------------------------------------------------------
// PID Control Task
//---------------------------------------------------------------------
void pidControlTask(void *parameter) {
  // This task runs continuously every 100 ms.
  for (;;) {
    for (int i = 0; i < 4; i++) {
      // Compute speed using encoder counts
      inputs[i] = getEncoderSpeed(i);
      // Compute PID output based on current speed and desired setpoint
      pids[i]->Compute();
      // Update motor PWM based on PID output
      setMotorSpeed(i, outputs[i]);
    }

    // Debug print current measurements, setpoints, and outputs
    Serial0.print("Inputs: ");
    for (int i = 0; i < 4; i++) {
      Serial0.print(inputs[i]);
      Serial0.print("  ");
    }
    Serial0.print("Setpoints: ");
    for (int i = 0; i < 4; i++) {
      Serial0.print(setpoints[i]);
      Serial0.print("  ");
    }
    Serial0.print("Outputs: ");
    for (int i = 0; i < 4; i++) {
      Serial0.print(outputs[i]);
      Serial0.print("  ");
    }
    Serial0.println();

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

//---------------------------------------------------------------------
// Movement Pattern Task
//---------------------------------------------------------------------
void movementTask(void *parameter) {
  // This task cycles the setpoints: forward 1 sec, stop 1 sec, backward 1 sec.
  for (;;) {
    // Move forward
    for (int i = 0; i < 4; i++) {
      setpoints[i] = 100;
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // Stop
    for (int i = 0; i < 4; i++) {
      setpoints[i] = 0;
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // Move backward
    for (int i = 0; i < 4; i++) {
      setpoints[i] = -100;
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

//---------------------------------------------------------------------
// getEncoderSpeed Implementation
//---------------------------------------------------------------------
double getEncoderSpeed(int motorIndex) {
  long currentCount = 0;

  // Read current count from the appropriate encoder
  switch (motorIndex) {
    case 0:
      currentCount = encoder0.getCount();
      break;
    case 1:
      currentCount = encoder1.getCount();
      break;
    case 2:
      currentCount = encoder2.getCount();
      break;
    case 3:
      currentCount = encoder3.getCount();
      break;
    default:
      return 0;
  }

  // Calculate difference from previous count
  long diff = currentCount - prevCounts[motorIndex];
  // Update previous count for next iteration
  prevCounts[motorIndex] = currentCount;

  // If the PID task runs every 100 ms, then dt = 0.1 seconds.
  double dt = 0.1;
  double speed = diff / dt;  // Speed in counts per second

  return speed;
}

//---------------------------------------------------------------------
// setMotorSpeed Implementation
//---------------------------------------------------------------------
void setMotorSpeed(int motorIndex, double pwmValue) {
  // For forward motion, drive PWM on pwmA and set pwmB to 0.
  // For reverse motion, drive PWM on pwmB (with absolute value) and set pwmA to 0.
  if (pwmValue >= 0) {
    analogWrite(motors[motorIndex].pwmA, (int)pwmValue);
    analogWrite(motors[motorIndex].pwmB, 0);
  } else {
    analogWrite(motors[motorIndex].pwmA, 0);
    analogWrite(motors[motorIndex].pwmB, (int)(-pwmValue));
  }
}
