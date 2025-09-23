#include "motor_control.hpp"

float setpoints[4], inputs[4], outputs[4];
QuickPID pids[4] = {
    QuickPID(&inputs[0], &outputs[0], &setpoints[0]),
    QuickPID(&inputs[1], &outputs[1], &setpoints[1]),
    QuickPID(&inputs[2], &outputs[2], &setpoints[2]),
    QuickPID(&inputs[3], &outputs[3], &setpoints[3])
};

// 馬達控制函式：根據四個浮點數 (-30.0 ~ 30.0) 控制馬達
void control_motors(const float *values) {
    if (pid_enable) {
        for (int i = 0; i < 4; i++) {
            setpoints[i] = values[i];
            inputs[i] = encoders[i].getCount();
            pids[i].Compute();
            uint8_t pwmValue = (uint8_t)fabs(outputs[i]);
            if (pwmValue < 100.0) {
                pwmValue = 100.0;
            }
            if (outputs[i] > 0) {
                // 正轉
                ledcWrite(i, pwmValue);
                ledcWrite(i + 4, 0);
            } else if (outputs[i] < 0) {
                // 反轉
                ledcWrite(i, 0);
                ledcWrite(i + 4, pwmValue);
            } else {
                // 停止
                ledcWrite(i, 0);
                ledcWrite(i + 4, 0);
            }
        }
    } else {
        for (int i = 0; i < 4; i++) {
            float value = values[i];
            uint8_t pwmValue = (uint8_t)(fabs(value) / 30.0 * 255.0f);
            if (pwmValue < 100.0) {
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
}

void motor_test(){
    while(1){
        Serial0.println("Testing motors forward");
        for (int i = 0; i < 4; i++) {
            ledcWrite(i, 150);
            ledcWrite(i + 4, 0);
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);

        Serial0.println("Testing motors backward");
        for (int i = 0; i < 4; i++) {
            ledcWrite(i, 0);
            ledcWrite(i + 4, 150);
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);

        Serial0.println("Stopping motors");
        for (int i = 0; i < 4; i++) {
            ledcWrite(i, 0);
            ledcWrite(i + 4, 0);
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);
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

void setup_encoders() {
    for (int i = 0; i < 4; i++) {
        encoders[i].attachHalfQuad(motors[i].encA, motors[i].encB);
        encoders[i].clearCount();
        pids[i].SetTunings(Kp, Ki, Kd);
        pids[i].SetOutputLimits(-255, 255);
        pids[i].SetSampleTimeUs(10000);
        pids[i].SetMode(1);
    }
}
