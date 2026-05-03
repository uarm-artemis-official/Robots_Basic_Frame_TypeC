
void init_auto_aim_apps() {
    // Set communication node ID for auto aim rig (gimbal board)
    communication.set_node_id(simple_comm::NodeID::Gimbal);

    ASSERT(rtos.task_create(
               gimbal_task_handle, const_cast<char*>("GimbalTask"),
               [](void* arg) { gimbal_app.run(static_cast<const void*>(arg)); },
               nullptr, 512, MW_RTOS::TaskPriority::Realtime),
           "Failed to create GimbalTask.");

    ASSERT(rtos.task_create(
               imu_task_handle, const_cast<char*>("IMUTask"),
               [](void* arg) { imu_app.run(static_cast<const void*>(arg)); },
               nullptr, 256, MW_RTOS::TaskPriority::Realtime),
           "Failed to create IMUTask.");

    ASSERT(rtos.task_create(
               timer_task_handle, const_cast<char*>("TimerTask"),
               [](void* arg) { timer_app.run(static_cast<const void*>(arg)); },
               nullptr, 256, MW_RTOS::TaskPriority::High),
           "Failed to create TimerTask.");

    ASSERT(rtos.task_create(
               rc_task_handle, const_cast<char*>("RCTask"),
               [](void* arg) { rc_app.run(static_cast<const void*>(arg)); },
               nullptr, 384, MW_RTOS::TaskPriority::High),
           "Failed to create RCTask.");
}
