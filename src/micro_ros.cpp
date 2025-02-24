#include "micro_ros.hpp"

std_msgs__msg__Float32MultiArray msg;
// std_msgs__msg__Int64MultiArray msg;
rcl_subscription_t subscriber;
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

    RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "car_C_rear_wheel"));

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

void subscription_callback(const void *msgin) {
    const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msgin;
    targetVelBuffer[0] = msg->data.data[0];
    targetVelBuffer[1] = msg->data.data[1];
    // digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH);
}

void destroy_entities() {
    targetVelBuffer[0] = 0.0;
    targetVelBuffer[1] = 0.0;
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    // 銷毀其他實體
    rcl_subscription_fini(&subscriber, &node);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);

    // 手動釋放動態分配的內存
    if (msg.data.data != NULL) {
        free(msg.data.data);
        msg.data.data = NULL;  // 避免懸空指針
    }

    if (msg.layout.dim.data != NULL) {
        // 如果有嵌套的分配，先釋放內部
        for (size_t i = 0; i < msg.layout.dim.capacity; i++) {
            if (msg.layout.dim.data[i].label.data != NULL) {
                free(msg.layout.dim.data[i].label.data);
                msg.layout.dim.data[i].label.data = NULL;
            }
        }
        // 再釋放外層的指針
        free(msg.layout.dim.data);
        msg.layout.dim.data = NULL;
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
                };
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
