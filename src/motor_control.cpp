#include "motor_control.hpp"

// 馬達控制函式：根據四個浮點數 (-30 ~ 30) 控制馬達
void control_motors(const float *values) {
  for (int i = 0; i < 4; i++) {
    float value = values[i];  // 輸入範圍 -30 ~ 30
    // 計算 PWM 值：當 abs(value)==30 時，輸出 255
    uint8_t pwmValue = (uint8_t)(fabs(value) / 30.0f * 255.0f);
    // 可選：如果有可能超出 30，則可加上 clamping 保護
    if (pwmValue > 255) pwmValue = 255;

    if (value > 0) {
      // 正轉：正轉通道輸出 pwmValue，反轉通道關閉
      ledcWrite(i, pwmValue);
      ledcWrite(i + 4, 0);
    } else if (value < 0) {
      // 反轉：反轉通道輸出 pwmValue，正轉通道關閉
      ledcWrite(i, 0);
      ledcWrite(i + 4, pwmValue);
    } else {
      // 停止：兩邊均關閉
      ledcWrite(i, 0);
      ledcWrite(i + 4, 0);
    }
  }
}

void setup_motor_pwm() {
    for (int i = 0; i < 4; i++) {
        ledcSetup(i, pwmFreq, pwmResolution);
        ledcAttachPin(motors[i].pwmA, i);
        ledcSetup(i + 4, pwmFreq, pwmResolution);
        ledcAttachPin(motors[i].pwmB, i + 4);
    }
}
