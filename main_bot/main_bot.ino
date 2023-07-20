#include <yaml.h>
#include <micro_ros_arduino.h>

#include "mainbot_core_config.h"


void setup() {
  set_microros_transports();

  motordriver.init();
  sensors.init();

  sensors.updateIMUinit();
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  // create node
  RCCHECK(rclc_node_init_default(&node, "main_bot_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &tf_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
    "/mainbot_tf"));

  RCCHECK(rclc_publisher_init_default(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/mainbot_imu"));

  RCCHECK(rclc_publisher_init_default(
    &motor_angvel_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Quaternion),
    "/mainbot_motor_angularVel"));

  RCCHECK(rclc_publisher_init_default(
    &motor_pos_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Quaternion),
    "/mainbot_motor_encoderPos"));


  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Quaternion),
    "/mainbot_target_velocity"));

  const unsigned int timer_timeout = 1000;

  RCCHECK(rclc_timer_init_default(
    &timer_motor_angvel,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback_motor_angvel));

  RCCHECK(rclc_timer_init_default(
    &timer_motor_pos,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback_motor_pos));

  //executor init
  RCCHECK(rclc_executor_init(&executor_tf_pub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_init(&executor_imu_pub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_init(&executor_motor_angvel_pub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_init(&executor_motor_pos_pub, &support.context, 1, &allocator));


  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &target_velocity, &target_vel_subscription_callback, ON_NEW_DATA));


  tf_message_init();

}

void loop() {
  struct timespec tv = {0};
  clock_gettime(0, &tv);

  //TF_node, imu pub
  sensors.updateIMU();
  double q[4];
  sensors.euler_to_quat(q);
  main_bot_imu = sensors.getIMU();

  tf_message->transforms.data[0].transform.rotation.x = (double) q[1];
	tf_message->transforms.data[0].transform.rotation.y = (double) q[2];
	tf_message->transforms.data[0].transform.rotation.z = (double) q[3];
	tf_message->transforms.data[0].transform.rotation.w = (double) q[0];
  tf_message->transforms.data[0].transform.translation.x = main_bot_imu.orientation.x;
	tf_message->transforms.data[0].transform.translation.y = main_bot_imu.orientation.y;
	tf_message->transforms.data[0].transform.translation.z = main_bot_imu.orientation.z;
  tf_message->transforms.data[0].header.stamp.nanosec = tv.tv_nsec;
	tf_message->transforms.data[0].header.stamp.sec = tv.tv_sec;

  main_bot_imu.header.stamp.nanosec = tv.tv_nsec;
  main_bot_imu.header.stamp.sec = tv.tv_sec;

  motor_pos.x=FR_encoderPos*360./90./12./2.;
  motor_pos.y=FL_encoderPos*360./90./12./2.;
  motor_pos.z=BR_encoderPos*360./90./12./2.;
  motor_pos.w=BL_encoderPos*360./90./12./2.;


  motor_angvel.x=FR_angelVel;
  motor_angvel.y=FL_angelVel;
  motor_angvel.z=BR_angelVel;
  motor_angvel.w=BL_angelVel;

  //pub
  RCSOFTCHECK(rcl_publish(&tf_publisher, tf_message, NULL));
  RCSOFTCHECK(rcl_publish(&imu_publisher, &main_bot_imu, NULL));
  RCSOFTCHECK(rcl_publish(&motor_pos_publisher, &motor_pos, NULL));
  RCSOFTCHECK(rcl_publish(&motor_angvel_publisher, &motor_angvel, NULL));


  //target_velocity sub
  RCSOFTCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100)));


}
