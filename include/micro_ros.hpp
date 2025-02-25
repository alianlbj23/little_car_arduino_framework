#ifndef _MICRO_ROS_HPP_
#define _MICRO_ROS_HPP_
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <micro_ros_utilities/type_utilities.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int64_multi_array.h>
#include <stdio.h>

#include "params.hpp"

extern std_msgs__msg__Float32MultiArray msg;
// extern std_msgs__msg__Int64MultiArray msg;
extern rcl_subscription_t subscriber;
extern rclc_executor_t executor;
extern rclc_support_t support;
extern rcl_allocator_t allocator;
extern rcl_node_t node;

extern enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

#define RCCHECK(fn)                    \
    do {                               \
        rcl_ret_t temp_rc = fn;        \
        if ((temp_rc != RCL_RET_OK)) { \
            while (1) {                \
                delay(100);            \
            }                          \
        }                              \
    } while (0)
#define RCSOFTCHECK(fn)                \
    do {                               \
        rcl_ret_t temp_rc = fn;        \
        if ((temp_rc != RCL_RET_OK)) { \
        }                              \
    } while (0)
#define EXECUTE_EVERY_N_MS(MS, X)          \
    do {                                   \
        static volatile int64_t init = -1; \
        if (init == -1) {                  \
            init = uxr_millis();           \
        }                                  \
        if (uxr_millis() - init > MS) {    \
            X;                             \
            init = uxr_millis();           \
        }                                  \
    } while (0)

bool create_entities();
void destroy_entities();
void MicroROSWheel(void *pvParameters);
void subscription_callback(const void *msgin);

#endif
