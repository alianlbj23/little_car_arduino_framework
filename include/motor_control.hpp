#ifndef _MOTOR_CONTROL_HPP_
#define _MOTOR_CONTROL_HPP_

#include <Arduino.h>
#include <cmath>  // fabs()
#include "params.hpp"  // 需要馬達 GPIO 設定

// 馬達控制函式：根據四個浮點數 (-1.0 ~ 1.0) 控制馬達
void control_motors(const float *values);
void setup_motor_pwm();
void motor_test();
#endif // _MOTOR_CONTROL_HPP_
