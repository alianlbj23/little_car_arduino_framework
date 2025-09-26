#include "motor_control.hpp"

float setpoints[4], inputs[4], outputs[4];
QuickPID pids[4] = {
    QuickPID(&inputs[0], &outputs[0], &setpoints[0]),
    QuickPID(&inputs[1], &outputs[1], &setpoints[1]),
    QuickPID(&inputs[2], &outputs[2], &setpoints[2]),
    QuickPID(&inputs[3], &outputs[3], &setpoints[3])
};

volatile long prev_counts[4] = {0,0,0,0};
float speed_meas[4] = {0,0,0,0};   // 每10ms的delta ticks
float speed_set[4]  = {0,0,0,0};   // 每10ms的目標delta ticks

constexpr float SPEED_SCALE = 3.0f;  // 可調參數

// 新增：只設定目標值的函式（由 ROS subscriber 呼叫）
void set_motor_targets(const float *values) {
    for (int i = 0; i < 4; i++) {
        // 將 -30..30 映射為每10ms的目標增量（tick/10ms）
        speed_set[i] = values[i] * SPEED_SCALE;
        setpoints[i] = speed_set[i];       // 與 PID 的 setpoints 對齊
    }
}

void pid_control_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms
    
    while (1) {
        if (pid_enable) {
            for (int i = 0; i < 4; i++) {
                long now = encoders[i].getCount();
                long delta = now - prev_counts[i];
                prev_counts[i] = now;

                // 量測改為「每10ms的增量 ticks」
                speed_meas[i] = (float)delta;

                // 餵給 PID 的 inputs / setpoints 必須是同單位
                inputs[i]    = speed_meas[i];
                setpoints[i] = speed_set[i];

                pids[i].Compute();

                // 將 PID 輸出（-255..255）轉為雙向PWM
                int pwm = (int)roundf(outputs[i]);

                // 最小啟動PWM（克服靜摩擦）
                if (pwm != 0) {
                    int mag = abs(pwm);
                    if (mag < 100) mag = 100;
                    if (mag > 255) mag = 255;

                    if (pwm > 0) {
                        ledcWrite(i, mag);
                        ledcWrite(i + 4, 0);
                    } else {
                        ledcWrite(i, 0);
                        ledcWrite(i + 4, mag);
                    }
                } else {
                    ledcWrite(i, 0);
                    ledcWrite(i + 4, 0);
                }
            }
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}


// 馬達控制函式：根據四個浮點數 (-30.0 ~ 30.0) 控制馬達
// 保留向後兼容性，但現在建議使用 set_motor_targets + pid_control_task
void control_motors(const float *values) {
    if (pid_enable) {
        // 如果啟用 PID，只設定目標值，讓 PID 任務處理
        set_motor_targets(values);
    } else {
        // 開環控制模式（原有邏輯）
        for (int i = 0; i < 4; i++) {
            float value = values[i];
            uint8_t pwmValue = (uint8_t)(fabs(value) / 30.0 * 255.0f);
            if (pwmValue > 0 && pwmValue < 100) {
                pwmValue = 100;
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
        prev_counts[i] = 0;

        pids[i].SetTunings(Kp, Ki, Kd);     // 起手勢：Kp大於Ki，Ki小，Kd小
        pids[i].SetOutputLimits(-255, 255);
        pids[i].SetSampleTimeUs(10000);     // 10ms
        // 如果極性相反可改 REVERSE（通常 QuickPID: 0=REVERSE, 1=DIRECT）
        // pids[i].SetControllerDirection(QuickPID::DIRECT);
        pids[i].SetMode(1);
    }
}

