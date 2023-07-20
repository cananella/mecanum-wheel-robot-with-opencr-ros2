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
#include <sensor_msgs/msg/imu.h>



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


rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu main_bot_imu;
rclc_executor_t executor_imu_pub;

rcl_publisher_t tf_publisher;
tf2_msgs__msg__TFMessage * tf_message;
rclc_executor_t executor_tf_pub;


rcl_publisher_t motor_angvel_publisher;
geometry_msgs__msg__Quaternion motor_angvel;
rclc_executor_t executor_motor_angvel_pub;

rcl_publisher_t motor_pos_publisher;
geometry_msgs__msg__Quaternion motor_pos;
rclc_executor_t executor_motor_pos_pub;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer_motor_angvel;
rcl_timer_t timer_motor_pos;

rcl_subscription_t subscriber;
geometry_msgs__msg__Quaternion target_velocity;
rclc_executor_t executor_sub;


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

extern "C" int clock_gettime(clockid_t unused, struct timespec *tp);

extern int FR_encoderPos;
extern int FL_encoderPos;
extern int BR_encoderPos;
extern int BL_encoderPos;

extern float FR_angelVel;
extern float FL_angelVel;
extern float BR_angelVel;
extern float BL_angelVel;

void timer_callback_motor_angvel(rcl_timer_t * timer, int64_t last_call_time){
  RCLC_UNUSED(last_call_time);
  RCLC_UNUSED(timer);
  motor_angvel.x=FR_angelVel;
  motor_angvel.y=FL_angelVel;
  motor_angvel.z=BR_angelVel;
  motor_angvel.w=BL_angelVel;
  RCSOFTCHECK(rcl_publish(&motor_angvel_publisher, &motor_angvel, NULL));

}

void timer_callback_motor_pos(rcl_timer_t * timer, int64_t last_call_time){
  RCLC_UNUSED(last_call_time);
  RCLC_UNUSED(timer);
  motor_pos.x=FR_encoderPos*360./90./12./2.;
  motor_pos.y=FL_encoderPos*360./90./12./2.;
  motor_pos.z=BR_encoderPos*360./90./12./2.;
  motor_pos.w=BL_encoderPos*360./90./12./2.;
  RCSOFTCHECK(rcl_publish(&motor_pos_publisher, &motor_pos, NULL));
}

void error_loop(){
  while(1){
    DEBUG_SERIAL.print("error");
  }
}

void target_vel_subscription_callback(const void * msgin)
{
  const geometry_msgs__msg__Quaternion * target_velocity = (const geometry_msgs__msg__Quaternion *)msgin;
  double x = target_velocity->x;
  double y = target_velocity->y;
  double w = target_velocity->w;
  if (x==0.0 && y==0.0 && w==0.0){
    motordriver.Stop();
  }
  else{
    motordriver.control_vel(x,y,w);
  }
}

void tf_message_init(){

  tf_message = tf2_msgs__msg__TFMessage__create();
  geometry_msgs__msg__TransformStamped__Sequence__init(&tf_message->transforms, 1);

  tf_message->transforms.data[0].header.frame_id.data = (char*)malloc(100*sizeof(char));
  char string1[] = "/main_bot";
  memcpy(tf_message->transforms.data[0].header.frame_id.data, string1, strlen(string1) + 1);
  tf_message->transforms.data[0].header.frame_id.size = strlen(tf_message->transforms.data[0].header.frame_id.data);
  tf_message->transforms.data[0].header.frame_id.capacity = 100;

  char string2[] = "/inertial_unit";
  tf_message->transforms.data[0].child_frame_id.data =  (char*)malloc(100*sizeof(char));
  memcpy(tf_message->transforms.data[0].child_frame_id.data, string2, strlen(string2) + 1);
  tf_message->transforms.data[0].child_frame_id.size = strlen(tf_message->transforms.data[0].child_frame_id.data);
  tf_message->transforms.data[0].child_frame_id.capacity = 100;

}


#endif
