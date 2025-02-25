#include "micro_ros.hpp"
#include "motor_control.hpp"

std_msgs__msg__Float32MultiArray msg;          // 訂閱用訊息
std_msgs__msg__Float32MultiArray wheel_info_msg; // 發佈用訊息

rcl_subscription_t subscriber;
rcl_publisher_t wheel_info_pub;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
enum states state;

bool create_entities() {
    allocator = rcl_get_default_allocator();

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 1);
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

    // 初始化訂閱者 "car_C_rear_wheel"
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "car_C_rear_wheel"));

    // 為訂閱用訊息分配記憶體
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

    // 初始化 publisher "wheel_info"
    RCCHECK(rclc_publisher_init_default(
        &wheel_info_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "wheel_info"));

    // 為發佈用訊息分配記憶體
    wheel_info_msg.data.capacity = 10;
    wheel_info_msg.data.size = 0;
    wheel_info_msg.data.data = (float *)malloc(wheel_info_msg.data.capacity * sizeof(float));
    wheel_info_msg.layout.dim.capacity = 10;
    wheel_info_msg.layout.dim.size = 0;
    wheel_info_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension *)malloc(wheel_info_msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));
    for (size_t i = 0; i < wheel_info_msg.layout.dim.capacity; i++) {
        wheel_info_msg.layout.dim.data[i].label.capacity = 10;
        wheel_info_msg.layout.dim.data[i].label.size = 0;
        wheel_info_msg.layout.dim.data[i].label.data = (char *)malloc(wheel_info_msg.layout.dim.data[i].label.capacity * sizeof(char));
    }

    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

    return true;
}

void subscription_callback(const void *msgin) {
    // 將接收到的訊息轉型
    const std_msgs__msg__Float32MultiArray *incoming_msg = (const std_msgs__msg__Float32MultiArray *)msgin;
    // motor_test();
    // 控制馬達（依照 -30 ~ 30 的數值）
    control_motors(incoming_msg->data.data);

    // 將接收到的數值複製到 wheel_info_msg 中（可依需求修改或轉換數據）
    wheel_info_msg.data.size = incoming_msg->data.size;
    memcpy(wheel_info_msg.data.data, incoming_msg->data.data, incoming_msg->data.size * sizeof(float));

    // 發佈 wheel_info 訊息
    RCCHECK(rcl_publish(&wheel_info_pub, &wheel_info_msg, NULL));
}

void destroy_entities() {
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    // 銷毀訂閱者、發佈者及其他實體
    rcl_subscription_fini(&subscriber, &node);
    rclc_executor_fini(&executor);
    rcl_publisher_fini(&wheel_info_pub, &node);
    rcl_node_fini(&node);
    rclc_support_fini(&support);

    // 釋放訂閱訊息記憶體
    if (msg.data.data != NULL) {
        free(msg.data.data);
        msg.data.data = NULL;
    }
    if (msg.layout.dim.data != NULL) {
        for (size_t i = 0; i < msg.layout.dim.capacity; i++) {
            if (msg.layout.dim.data[i].label.data != NULL) {
                free(msg.layout.dim.data[i].label.data);
                msg.layout.dim.data[i].label.data = NULL;
            }
        }
        free(msg.layout.dim.data);
        msg.layout.dim.data = NULL;
    }

    // 釋放發佈訊息記憶體
    if (wheel_info_msg.data.data != NULL) {
        free(wheel_info_msg.data.data);
        wheel_info_msg.data.data = NULL;
    }
    if (wheel_info_msg.layout.dim.data != NULL) {
        for (size_t i = 0; i < wheel_info_msg.layout.dim.capacity; i++) {
            if (wheel_info_msg.layout.dim.data[i].label.data != NULL) {
                free(wheel_info_msg.layout.dim.data[i].label.data);
                wheel_info_msg.layout.dim.data[i].label.data = NULL;
            }
        }
        free(wheel_info_msg.layout.dim.data);
        wheel_info_msg.layout.dim.data = NULL;
    }
}

void MicroROSWheel(void *pvParameters) {
    set_microros_serial_transports(Serial0);
    state = WAITING_AGENT;
    while (1) {
        switch (state) {
            case WAITING_AGENT:
                EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
                break;
            case AGENT_AVAILABLE:
                state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
                if (state == WAITING_AGENT) {
                    destroy_entities();
                }
                break;
            case AGENT_CONNECTED:
                EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
                if (state == AGENT_CONNECTED) {
                    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
                }
                break;
            case AGENT_DISCONNECTED:
                destroy_entities();
                state = WAITING_AGENT;
                break;
            default:
                break;
        }
    }
}
