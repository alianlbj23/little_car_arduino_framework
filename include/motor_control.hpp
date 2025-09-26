#ifndef _MOTOR_CONTROL_HPP_
#define _MOTOR_CONTROL_HPP_

#include <Arduino.h>
#include <cmath>  // fabs()
#include "params.hpp"  // 需要馬達 GPIO 設定
#include "QuickPID.h"

// 馬達控制函式：根據四個浮點數 (-30.0 ~ 30.0) 控制馬達
void control_motors(const float *values);
void set_motor_targets(const float *values);  // 只設定目標值
void pid_control_task(void *pvParameters);    // PID 持續控制任務
void setup_motor_pwm();
void setup_encoders();
void motor_test();

extern float setpoints[4], inputs[4], outputs[4];
extern QuickPID pids[4];

#endif // _MOTOR_CONTROL_HPP_
