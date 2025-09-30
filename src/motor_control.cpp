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

constexpr float SPEED_SCALE = 3.0f;         // 可調參數：輸入 -> 目標增量
constexpr int PWM_MIN_OUTPUT = 100;         // 克服靜摩擦的最小PWM
constexpr int PID_STOP_THRESHOLD = 15;      // PID輸出小於此視為停止
constexpr long ENCODER_NOISE_THRESHOLD = 1; // 絕對增量小於等於此值視為雜訊
constexpr float SPEED_STOP_BAND = 5.0f;     // 目標/回授均落在此區間則直接停車

constexpr int   ENC_PPR    = 11;     // 編碼器每圈脈衝數（看 datasheet）
constexpr int   QUAD_MULT  = 4;      // 正交解碼倍率 (FullQuad = x4)
constexpr float GEAR_RATIO = 30.0f;  // 減速比 (馬達軸 → 輪子)
constexpr float LOOP_DT    = 0.01f;  // 控制週期 (10ms)

constexpr float TICKS_PER_MOTOR_REV = ENC_PPR * QUAD_MULT;
constexpr float TICKS_PER_WHEEL_REV = TICKS_PER_MOTOR_REV * GEAR_RATIO;
// 新增：只設定目標值的函式（由 ROS subscriber 呼叫）

void set_motor_targets(const float *values) {
    for (int i = 0; i < 4; i++) {
        // 將 -30..30 映射為每10ms的目標增量（tick/10ms）
        // float target = values[i] * SPEED_SCALE;
        float rpm = values[i];
        float target_ticks = rpm * (TICKS_PER_WHEEL_REV / 6000.0f);
        if (fabs(target_ticks) <= SPEED_STOP_BAND) {
            target_ticks = 0.0f;
        }
        speed_set[i] = target_ticks;
        setpoints[i] = target_ticks;       // 與 PID 的 setpoints 對齊
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

                if (delta <= ENCODER_NOISE_THRESHOLD && delta >= -ENCODER_NOISE_THRESHOLD) {
                    delta = 0;
                }

                // 量測改為「每10ms的增量 ticks」
                speed_meas[i] = static_cast<float>(delta);

                float measurement = speed_meas[i];
                float target = speed_set[i];

                if (fabs(target) <= SPEED_STOP_BAND) {
                    speed_set[i] = 0.0f;
                    inputs[i] = 0.0f;
                    setpoints[i] = 0.0f;
                    outputs[i] = 0.0f;
                    pids[i].Reset();
                    ledcWrite(i, 0);
                    ledcWrite(i + 4, 0);
                    continue;
                }

                // 餵給 PID 的 inputs / setpoints 必須是同單位
                inputs[i]    = measurement;
                setpoints[i] = target;

                pids[i].Compute();

                // 將 PID 輸出（-255..255）轉為雙向PWM
                int pwm = (int)roundf(outputs[i]);

                if (abs(pwm) < PID_STOP_THRESHOLD) {
                    ledcWrite(i, 0);
                    ledcWrite(i + 4, 0);
                    continue;
                }

                int mag = abs(pwm);
                if (mag < PWM_MIN_OUTPUT) {
                    mag = PWM_MIN_OUTPUT;
                } else if (mag > 255) {
                    mag = 255;
                }

                if (pwm > 0) {
                    ledcWrite(i, mag);
                    ledcWrite(i + 4, 0);
                } else {
                    ledcWrite(i, 0);
                    ledcWrite(i + 4, mag);
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
            float pwmFloat = fabs(value) / 30.0f * 255.0f;
            if (pwmFloat > 0 && pwmFloat < PWM_MIN_OUTPUT) {
                pwmFloat = PWM_MIN_OUTPUT;
            }
            if (pwmFloat > 255.0f) {
                pwmFloat = 255.0f;
            }
            uint8_t pwmValue = static_cast<uint8_t>(pwmFloat);
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

inline float ticks10ms_to_rpm(float t10) {
    return t10 * (6000.0f / TICKS_PER_WHEEL_REV);
}

// 掃描 PWM，測量對應的 RPM
void pwm_rpm_test() {
    int pwm_values[] = {50, 100, 150, 200, 255, 500};  // 想測的 PWM 點
    int n = sizeof(pwm_values) / sizeof(pwm_values[0]);

    for (int p=0; p<n; p++) {
        int pwm = pwm_values[p];
        Serial0.printf("\n=== Test PWM = %d ===\n", pwm);

        // 設定 PWM，正轉
        for (int i=0; i<4; i++) {
            ledcWrite(i, pwm);
            ledcWrite(i+4, 0);
        }

        // 等待馬達穩定
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        // 測量 1 秒內的 ticks
        long start_counts[4];
        for (int i=0; i<4; i++) {
            start_counts[i] = encoders[i].getCount();
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);  // 量測 1 秒

        for (int i=0; i<4; i++) {
            long delta = encoders[i].getCount() - start_counts[i];
            float ticks_per_10ms = (float)delta / 100.0f; // 1 秒 = 100*10ms
            float rpm = ticks10ms_to_rpm(ticks_per_10ms);
            Serial0.printf("Motor %d: %ld ticks, %.2f ticks/10ms, %.2f rpm at PWM %d\n", 
                          i, delta, ticks_per_10ms, rpm, pwm);
        }

        // 停止馬達
        for (int i=0; i<4; i++) {
            ledcWrite(i, 0);
            ledcWrite(i+4, 0);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
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

void motor_test_pid(){
    while(1){
        Serial0.println("PID Test: Forward 350 rpm");
        float forward_targets[4] = {350,350,350,350};
        set_motor_targets(forward_targets);

        // 等控制建立起來，再每200ms印一次
        vTaskDelay(300 / portTICK_PERIOD_MS);
        for (int t=0; t<15; t++) { // 約3秒
            vTaskDelay(200 / portTICK_PERIOD_MS);
            Serial0.println("Current RPM:");
            for (int i=0;i<4;i++){
                float rpm = ticks10ms_to_rpm(speed_meas[i]); // 不再動 prev_counts！
                Serial0.printf("Motor %d: %.2f rpm\n", i, rpm);
            }
        }

        Serial0.println("PID Test: Backward -300 rpm");
        float backward_targets[4] = {-300,-300,-300,-300};
        set_motor_targets(backward_targets);
        vTaskDelay(300 / portTICK_PERIOD_MS);
        for (int t=0; t<15; t++) {
            vTaskDelay(200 / portTICK_PERIOD_MS);
            Serial0.println("Current RPM:");
            for (int i=0;i<4;i++){
                float rpm = ticks10ms_to_rpm(speed_meas[i]);
                Serial0.printf("Motor %d: %.2f rpm\n", i, rpm);
            }
        }

        Serial0.println("PID Test: Stop");
        float stop_targets[4] = {0,0,0,0};
        set_motor_targets(stop_targets);
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
        encoders[i].attachFullQuad(motors[i].encA, motors[i].encB);
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
