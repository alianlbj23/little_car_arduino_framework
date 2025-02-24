#ifndef _PARAMS_HPP_
#define _PARAMS_HPP_

#include <cstdint>

// Define the reverse condition
#define REVERSE_PINS   // Uncomment this line to reverse the pins
#define MOTOR_COUNT 4  // Updated to support 4 motors

// Define motor velocity buffers
extern double targetVelBuffer[MOTOR_COUNT];
extern double currentVelsBuffer[MOTOR_COUNT];

// Define encoder resolutions for each motor
const uint16_t ENCODER_RESOLUTIONS[MOTOR_COUNT] = {330, 350, 330, 350};

/*--------------------------------------------------*/
/*--------------- Pin Definition -------------------*/
/*--------------------------------------------------*/


// Servo
#define PIN_SERVO 4

// Define the pins based on Yahboom micro-ROS board
#ifndef REVERSE_PINS
// Motor1
#define PIN_MOTOR_P1 21
#define PIN_MOTOR_P2 17
// Motor2
#define PIN_MOTOR2_P1 22
#define PIN_MOTOR2_P2 23
// Motor3
#define PIN_MOTOR3_P1 18
#define PIN_MOTOR3_P2 19
// Motor4
#define PIN_MOTOR4_P1 5
#define PIN_MOTOR4_P2 25

// Encoder1
#define PIN_ENCODER_P1 34
#define PIN_ENCODER_P2 35
// Encoder2
#define PIN_ENCODER2_P1 16
#define PIN_ENCODER2_P2 27
// Encoder3
#define PIN_ENCODER3_P1 36
#define PIN_ENCODER3_P2 39
// Encoder4
#define PIN_ENCODER4_P1 32
#define PIN_ENCODER4_P2 33

#else
// Reverse pin definitions
#define PIN_MOTOR_P1 4
#define PIN_MOTOR_P2 5
#define PIN_MOTOR2_P1 15
#define PIN_MOTOR2_P2 16
#define PIN_MOTOR3_P1 9
#define PIN_MOTOR3_P2 10
#define PIN_MOTOR4_P1 13
#define PIN_MOTOR4_P2 14

#define PIN_ENCODER_P1 6
#define PIN_ENCODER_P2 7
#define PIN_ENCODER2_P1 47
#define PIN_ENCODER2_P2 48
#define PIN_ENCODER3_P1 11
#define PIN_ENCODER3_P2 12
#define PIN_ENCODER4_P1 1
#define PIN_ENCODER4_P2 2
#endif

// Motor enable pins
#define PIN_MOTOR_ENA 1
#define PIN_MOTOR2_ENA 2
#define PIN_MOTOR3_ENA 3
#define PIN_MOTOR4_ENA 4

/*--------------------------------------------------*/
/*--------------- Const Definition -----------------*/
/*--------------------------------------------------*/

// Servo
#define SERVO_MIN 60
#define SERVO_MAX 120
#define SERVO_INIT 90
#define SERVO_DELAY 100

// Queue
#define QUEUE_SIZE 10

// Time
#define QUEUE_TIMEOUT 5
#define ENCODER_DELAY 10
#define SERIAL_READ_DELAY 10
#define SERIAL_WRITE_DELAY 10
#define MOTOR_DELAY 10

// Math
#define PI 3.14159

// PID
#define PID_DELAY 100

// Define PID constants
static constexpr double Kp = 5.0;
static constexpr double Ki = 65.25;
static constexpr double Kd = 0.001;

#endif