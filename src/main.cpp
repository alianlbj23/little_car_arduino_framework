#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "micro_ros.hpp"
#include "motor_control.hpp"

// 使用 Serial0 作為 UART 傳輸與除錯的初始化
void init_serial0() {
  Serial0.begin(115200);
  delay(100);
}

void motor_test_task(void *pvParameters) {
    motor_test();
}

void motor_test_pid_task(void *pvParameters) {
    motor_test_pid();
}

void setup() {
  // 初始化 Serial0
    init_serial0();
//   Serial0.println("啟動 micro-ROS 與馬達控制程式 (使用 UART)");

    // 設定馬達 PWM 腳位：正轉與反轉分別用不同的 LEDC 通道
    setup_motor_pwm();
    setup_encoders();
    
    // 啟動 micro-ROS 任務
    xTaskCreatePinnedToCore(MicroROSWheel, "MicroROSWheel", 81192 ,NULL, 1, NULL, 1);

    // PID test
    // xTaskCreatePinnedToCore(motor_test_pid_task, "motor_test_pid_task", 4096, NULL, 1, NULL, 1);
    
    // 啟動 PID 控制任務 (高優先級，精確 10ms 週期)
    xTaskCreatePinnedToCore(pid_control_task, "PIDControl", 4096, NULL, 2, NULL, 0);
    
    // xTaskCreatePinnedToCore(motor_test_task, "motor_test_task", 4096, NULL, 1, NULL, 1);
    delay(100);
}

void loop() {
  // loop() 保留空白，由 FreeRTOS 任務持續處理 micro-ROS 與馬達控制
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
