#include "main.h"
#include "mobility_config.h"
#include "mobility_drive.h"
#include "ros_interface.h"
#include "sensors.h"
#include <math.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/bool.h>
#include <rosidl_runtime_c/string_functions.h>

static mobility_drive_t drive;
static volatile uint32_t motor_time_ms;
static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;
static rcl_subscription_t command_sub;
static rcl_publisher_t pubs[4]; /* IMU, GPS, physical e-stop, mobility heartbeat. */
static rcl_timer_t sensor_timer, heartbeat_timer;
static geometry_msgs__msg__Twist command;
static sensor_msgs__msg__Imu imu;
static sensor_msgs__msg__NavSatFix gps;
static bool messages_ready, support_ready, node_ready, sub_ready, executor_ready;
static bool timer_ready[2], pub_ready[4];

bool mobility_estop_asserted(void)
{
    return false;
}
static void write_motor(float value, uint32_t channel, GPIO_TypeDef *dir_port, uint16_t dir_pin, bool invert)
{
    if (invert) value = -value;
    /* Zero PWM before changing direction; no nonzero output survives a stop. */
    __HAL_TIM_SET_COMPARE(&htim4, channel, 0);
    HAL_GPIO_WritePin(dir_port, dir_pin, value >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim4, channel, (uint32_t)(fminf(1.0f, fabsf(value)) * (MOBILITY_PWM_PERIOD + 1U)));
}
void mobility_write_cytron(float left, float right)
{
    if (!MOBILITY_DRIVE_CONFIGURED || mobility_estop_asserted()) left = right = 0;
    /* 2x Cytron MDD10A: Left side (Front-Left PB6/PB4, Rear-Left PB7/PB5) */
    write_motor(left, MOTOR_FL_PWM_CHANNEL, MOTOR_FL_DIR_PORT, MOTOR_FL_DIR_PIN, MOBILITY_LEFT_INVERT);
    write_motor(left, MOTOR_RL_PWM_CHANNEL, MOTOR_RL_DIR_PORT, MOTOR_RL_DIR_PIN, MOBILITY_LEFT_INVERT);
    /* Right side (Front-Right PB8/PC13, Rear-Right PB9/PC14) */
    write_motor(right, MOTOR_FR_PWM_CHANNEL, MOTOR_FR_DIR_PORT, MOTOR_FR_DIR_PIN, MOBILITY_RIGHT_INVERT);
    write_motor(right, MOTOR_RR_PWM_CHANNEL, MOTOR_RR_DIR_PORT, MOTOR_RR_DIR_PIN, MOBILITY_RIGHT_INVERT);
    /* Motion active indicator (Yellow LED on PB14) */
    HAL_GPIO_WritePin(MOBILITY_LED2_PORT, MOBILITY_LED2_PIN,
        (fabsf(left) > 0.01f || fabsf(right) > 0.01f) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
void drive_init(void)
{
    if (!mobility_drive_init(&drive,MOBILITY_TRACK_WIDTH_M,MOBILITY_FULL_SPEED_MPS,
                             MOBILITY_COMMAND_TIMEOUT_MS,mobility_write_cytron)) Error_Handler();
}
void drive_stop(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    mobility_drive_stop(&drive); __set_PRIMASK(mask);
}
void motor_safety_tick(void)
{
    motor_time_ms += 10U;
    mobility_drive_safety_tick(&drive,motor_time_ms,mobility_estop_asserted());
}
static void command_callback(const void *input)
{
    const geometry_msgs__msg__Twist *msg=input;
    if (!msg || !isfinite(msg->linear.x) || !isfinite(msg->angular.z)) { drive_stop(); return; }
    float linear=fmaxf(-MOBILITY_MAX_LINEAR_MPS,fminf(MOBILITY_MAX_LINEAR_MPS,(float)msg->linear.x));
    float angular=fmaxf(-MOBILITY_MAX_ANGULAR_RPS,fminf(MOBILITY_MAX_ANGULAR_RPS,(float)msg->angular.z));
    /* Serialize a short setpoint update with the independent safety interrupt. */
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    mobility_drive_command(&drive,linear,angular,motor_time_ms,mobility_estop_asserted());
    __set_PRIMASK(mask);
}
static void stamp(builtin_interfaces__msg__Time *time)
{
    int64_t ns=rmw_uros_epoch_nanos();
    time->sec=(int32_t)(ns/1000000000LL); time->nanosec=(uint32_t)(ns%1000000000LL);
}
static void sensors_callback(rcl_timer_t *timer, int64_t last)
{
    (void)last;
    if (!timer) return;
    gps_poll();
    if (mobility_read_imu(&imu)) { stamp(&imu.header.stamp); (void)rcl_publish(&pubs[0],&imu,NULL); }
    if (mobility_read_gps(&gps)) {
        int64_t ns=rmw_uros_epoch_nanos()-(int64_t)(HAL_GetTick()-gps_sample_time_ms())*1000000LL;
        gps.header.stamp.sec=(int32_t)(ns/1000000000LL);
        gps.header.stamp.nanosec=(uint32_t)(ns%1000000000LL);
        (void)rcl_publish(&pubs[1],&gps,NULL);
    }
}
static void heartbeat_callback(rcl_timer_t *timer, int64_t last)
{
    (void)last;
    if (!timer) return;
    std_msgs__msg__Bool msg={.data=true};
    (void)rcl_publish(&pubs[3],&msg,NULL);
    msg.data=mobility_estop_asserted();
    (void)rcl_publish(&pubs[2],&msg,NULL);
}
#define CHECK(call) do { if ((call)!=RCL_RET_OK) goto fail; } while (0)
bool ros_entities_create(void)
{
    drive_stop();
    /* RCLC support/executor retain this pointer after setup returns. */
    static rcl_allocator_t allocator;
    allocator=rcl_get_default_allocator();
    rcl_init_options_t options=rcl_get_zero_initialized_init_options();
    if (rcl_init_options_init(&options,allocator)!=RCL_RET_OK) return false;
    rcl_ret_t code=rcl_init_options_set_domain_id(&options,MOBILITY_ROS_DOMAIN_ID);
    if (code==RCL_RET_OK) code=rclc_support_init_with_options(&support,0,NULL,&options,&allocator);
    (void)rcl_init_options_fini(&options);
    if (code!=RCL_RET_OK) return false;
    support_ready=true;
    /* Transport is up before this clock sync; drive is still zero. */
    if (rmw_uros_sync_session(1000)!=RMW_RET_OK) goto fail;
    CHECK(rclc_node_init_default(&node,"mobility_ecu_node","",&support)); node_ready=true;
    if (!messages_ready) {
        if (!geometry_msgs__msg__Twist__init(&command) || !sensor_msgs__msg__Imu__init(&imu) ||
            !sensor_msgs__msg__NavSatFix__init(&gps)) Error_Handler();
        if (!rosidl_runtime_c__String__assign(&imu.header.frame_id,"imu_link") ||
            !rosidl_runtime_c__String__assign(&gps.header.frame_id,"gps_link")) Error_Handler();
        messages_ready=true;
    }
    CHECK(rclc_subscription_init_best_effort(&command_sub,&node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg,Twist),"cmd_vel")); sub_ready=true;
    CHECK(rclc_publisher_init_best_effort(&pubs[0],&node,ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs,msg,Imu),"imu/data")); pub_ready[0]=true;
    CHECK(rclc_publisher_init_best_effort(&pubs[1],&node,ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs,msg,NavSatFix),"gps/fix")); pub_ready[1]=true;
    CHECK(rclc_publisher_init_default(&pubs[2],&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Bool),"estop")); pub_ready[2]=true;
    CHECK(rclc_publisher_init_default(&pubs[3],&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Bool),"mobility/heartbeat")); pub_ready[3]=true;
    CHECK(rclc_timer_init_default(&sensor_timer,&support,RCL_MS_TO_NS(20),sensors_callback)); timer_ready[0]=true;
    CHECK(rclc_timer_init_default(&heartbeat_timer,&support,RCL_MS_TO_NS(100),heartbeat_callback)); timer_ready[1]=true;
    executor=rclc_executor_get_zero_initialized_executor();
    CHECK(rclc_executor_init(&executor,&support.context,3,&allocator)); executor_ready=true;
    CHECK(rclc_executor_add_subscription(&executor,&command_sub,&command,command_callback,ON_NEW_DATA));
    CHECK(rclc_executor_add_timer(&executor,&sensor_timer));
    CHECK(rclc_executor_add_timer(&executor,&heartbeat_timer));
    return true;
fail:
    ros_entities_destroy(); return false;
}
void ros_entities_destroy(void)
{
    drive_stop();
    if (support_ready) (void)rmw_uros_set_context_entity_destroy_session_timeout(rcl_context_get_rmw_context(&support.context),0);
    if (executor_ready) { (void)rclc_executor_fini(&executor); executor_ready=false; }
    if (sub_ready) { (void)rcl_subscription_fini(&command_sub,&node); sub_ready=false; }
    for (unsigned i=0;i<4;i++) if (pub_ready[i]) { (void)rcl_publisher_fini(&pubs[i],&node); pub_ready[i]=false; }
    if (timer_ready[0]) { (void)rcl_timer_fini(&sensor_timer); timer_ready[0]=false; }
    if (timer_ready[1]) { (void)rcl_timer_fini(&heartbeat_timer); timer_ready[1]=false; }
    if (node_ready) { (void)rcl_node_fini(&node); node_ready=false; }
    if (support_ready) { (void)rclc_support_fini(&support); support_ready=false; }
    node=rcl_get_zero_initialized_node(); command_sub=rcl_get_zero_initialized_subscription();
    for (unsigned i=0;i<4;i++) pubs[i]=rcl_get_zero_initialized_publisher();
    sensor_timer=rcl_get_zero_initialized_timer(); heartbeat_timer=rcl_get_zero_initialized_timer();
}
void ros_spin(void) { (void)rclc_executor_spin_some(&executor,RCL_MS_TO_NS(5)); }
