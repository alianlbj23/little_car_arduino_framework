#include "params.hpp"

// 定義四個馬達對應的 GPIO 腳位
/*
M1 M3
M2 M4
*/
MotorConfig motors[4] = {
  {5, 4, 6, 7},      // M1
  {9, 10, 11, 12},   // M3
  {16, 15, 47, 48},  // M2
  {13, 14, 1, 2}     // M4
};

// PWM 設定：提高頻率到20kHz，8位解析度；LED 通道 0~3 控制正轉，4~7 控制反轉
const int pwmFreq = 20000;  // 20kHz - 更適合馬達控制
const int pwmResolution = 8;

bool pid_enable = true;

// PID gains
double Kp = 2.0, Ki = 0.08, Kd = 0.2;

ESP32Encoder encoders[4];
