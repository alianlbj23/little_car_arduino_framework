#include "motor_control.hpp"

// 馬達控制函式：根據四個浮點數 (-1.0 ~ 1.0) 控制馬達
void control_motors(const float *values) {
  for (int i = 0; i < 4; i++) {
    float value = values[i];
    uint8_t pwmValue = (uint8_t)(fabs(value) / 30.0 * 255.0f);
    if (pwmValue < 100.0){
        pwmValue = 100.0;
    }
    if (value > 0) {
      // 正轉
      ledcWrite(i, pwmValue);
      ledcWrite(i + 4, 0);
    } else if (value < 0) {
      // 反轉
      ledcWrite(i, 0);
      ledcWrite(i + 4, pwmValue);
    } else {
      // 停止
      ledcWrite(i, 0);
      ledcWrite(i + 4, 0);
    }
  }
}

void motor_test(){
    while(1){
        for (int i = 0; i < 4; i++) {
            ledcWrite(i, 50);
            ledcWrite(i + 4, 0);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            // ledcWrite(i, 0);
            // ledcWrite(i + 4, 0);
            // vTaskDelay(1000 / portTICK_PERIOD_MS);
            // ledcWrite(i, 0);
            // ledcWrite(i + 4, 200);
            // vTaskDelay(1000 / portTICK_PERIOD_MS);

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
