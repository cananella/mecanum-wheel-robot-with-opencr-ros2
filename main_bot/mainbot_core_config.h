#ifndef MAINBOT_CORE_CONFIG_H_
#define MAINBOT_CORE_CONFIG_H_


#include "include/motordriver.h"
#include "include/mainbot_sensor.h"

#include <micro_ros_arduino.h>
#include <IMU.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_msgs/msg/tf_message.h>
#include <geometry_msgs/msg/quaternion.h>



#define CONTROL_MOTOR_SPEED_FREQUENCY          30   //hz
#define CONTROL_MOTOR_TIMEOUT                  500  //ms
#define IMU_PUBLISH_FREQUENCY                  200  //hz
#define CMD_VEL_PUBLISH_FREQUENCY              30   //hz
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY    30   //hz
#define VERSION_INFORMATION_PUBLISH_FREQUENCY  1    //hz 
#define DEBUG_LOG_FREQUENCY                    10   //hz 

#define WHEEL_NUM                        4

#define FRONT_LEFT                             0
#define FRONT_RIGHT                            1
#define BACK_LEFT                              2
#define BACK_RIGHT                             3

#define LINEAR                           0
#define ANGULAR                          1

#define DEG2RAD(x)                       (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                       (x * 57.2957795131)  // *180/PI

MainbotMotorDriver motordriver;
MainbotSensor sensors;

rcl_publisher_t publisher;
tf2_msgs__msg__TFMessage * tf_message;
rclc_executor_t executor_pub;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

rcl_subscription_t subscriber;
geometry_msgs__msg__Quaternion target_velocity;
rclc_executor_t executor_sub;


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


#endif
