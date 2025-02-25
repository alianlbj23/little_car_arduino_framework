#ifndef _PARAMS_HPP_
#define _PARAMS_HPP_
#include <cstdint>

// 馬達配置結構
struct MotorConfig {
    uint8_t pwmA;  // 正轉 PWM 腳位
    uint8_t pwmB;  // 反轉 PWM 腳位
    uint8_t encA;
    uint8_t encB;
};


// 透過 extern 宣告全域變數（在 params.cpp 定義）
extern MotorConfig motors[4];
extern const int pwmFreq;
extern const int pwmResolution;

#endif // _PARAMS_HPP_
