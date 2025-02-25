#include "params.hpp"

// 定義四個馬達對應的 GPIO 腳位
MotorConfig motors[4] = {
  {4, 5, 6, 7},      // M1
  {15, 16, 47, 48},  // M2
  {9, 10, 11, 12},   // M3
  {13, 14, 1, 2}     // M4
};

// PWM 設定：5kHz, 8位解析度；LED 通道 0~3 控制正轉，4~7 控制反轉
const int pwmFreq = 5000;
const int pwmResolution = 8;
