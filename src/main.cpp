#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <stdlib.h>
#include <math.h>

// 宏定義錯誤檢查
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){} }

// 錯誤處理迴圈
void error_loop() {
  while (1) {
    // Serial0.println("micro-ROS 初始化發生錯誤！");
    delay(100);
  }
}

// 全域 micro-ROS 實體
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_subscription_t subscriber;
rclc_executor_t executor;
std_msgs__msg__Float32MultiArray msg;

// 訂閱回呼：根據接收到的四個浮點數控制四個馬達
void subscription_callback(const void * msgin)
{
  const std_msgs__msg__Float32MultiArray * incoming_msg = (const std_msgs__msg__Float32MultiArray *)msgin;

  // 檢查資料數量是否足夠 (至少 4 個數值)
  if (incoming_msg->data.size < 4) {
    // Serial0.println("Error: 接收到的陣列長度不足 4");
    return;
  }

  // 根據接收到的數值控制馬達（使用外部函式處理）
  extern void control_motors(const float *values);
  control_motors(incoming_msg->data.data);
}

// 馬達配置結構
struct MotorConfig {
  uint8_t pwmA;  // 正轉 PWM 腳位
  uint8_t pwmB;  // 反轉 PWM 腳位
  uint8_t encA;
  uint8_t encB;
};

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

// 馬達控制函式：根據四個浮點數 (範圍 -1.0 ~ 1.0) 控制馬達正轉、反轉或停止
void control_motors(const float *values) {
  for (int i = 0; i < 4; i++) {
    float value = values[i];
    uint8_t pwmValue = (uint8_t)(fabs(value) * 255.0f);
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

// 初始化 micro-ROS 實體的函式，包含節點、訂閱者與記憶體配置
bool create_entities() {
    allocator = rcl_get_default_allocator();

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, 1));  // 設定 domain_id 為 1
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "car_C_rear_wheel"));

    // 為 Float32MultiArray 配置資料記憶體
    msg.data.capacity = 10;
    msg.data.size = 0;
    msg.data.data = (float *)malloc(msg.data.capacity * sizeof(float));
    msg.layout.dim.capacity = 10;
    msg.layout.dim.size = 0;
    msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension *)malloc(msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));
    for (size_t i = 0; i < msg.layout.dim.capacity; i++) {
        msg.layout.dim.data[i].label.capacity = 10;
        msg.layout.dim.data[i].label.size = 0;
        msg.layout.dim.data[i].label.data = (char *)malloc(msg.layout.dim.data[i].label.capacity * sizeof(char));
    }

    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

    return true;
}

// 使用 Serial0 作為 UART 傳輸與除錯的初始化
void init_serial0() {
  Serial0.begin(115200);
  delay(2000);
}

void setup() {
  // 初始化 Serial0
  init_serial0();
//   Serial0.println("啟動 micro-ROS 與馬達控制程式 (使用 UART)");

  // 設定馬達 PWM 腳位：正轉與反轉分別用不同的 LEDC 通道
  for (int i = 0; i < 4; i++) {
    ledcSetup(i, pwmFreq, pwmResolution);
    ledcAttachPin(motors[i].pwmA, i);
    ledcSetup(i + 4, pwmFreq, pwmResolution);
    ledcAttachPin(motors[i].pwmB, i + 4);
  }

  // 初始化 micro-ROS 傳輸（注意：platformio.ini 需設定 transport = serial）
  set_microros_serial_transports(Serial0);

  // 建立 micro-ROS 節點、訂閱者與相關實體
  if (!create_entities()) {
    //   Serial0.println("建立 micro-ROS 實體失敗！");
      error_loop();
  }

  // 建立 FreeRTOS 任務以持續運行 micro-ROS executor
  xTaskCreate(
    [](void * parameter) {
      while(1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        vTaskDelay(100 / portTICK_PERIOD_MS);
      }
    },
    "micro_ros_executor_task",
    4096,
    NULL,
    1,
    NULL
  );
}

void loop() {
  // loop() 保留空白，由 FreeRTOS 任務持續處理 micro-ROS 與馬達控制
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
