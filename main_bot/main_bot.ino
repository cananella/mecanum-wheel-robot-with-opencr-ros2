#include "mainbot_core_config.h"

extern "C" int clock_gettime(clockid_t unused, struct timespec *tp);

void timer_callback(rcl_timer_t * timer, int64_t last_call_time){  
  RCLC_UNUSED(last_call_time);
  RCLC_UNUSED(timer);
}

void error_loop(){
  while(1){
    DEBUG_SERIAL.print("error");
  }
}

void subscription_callback(const void * msgin)
{  
  // const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  // digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH);  
}

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
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
    "/mainbot_tf"));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Quaternion),
    "/mainbot_target_velocity"));

  
  const unsigned int timer_timeout = 1000;

  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));

  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &target_velocity, &subscription_callback, ON_NEW_DATA));


  
  tf_message = tf2_msgs__msg__TFMessage__create();
  geometry_msgs__msg__TransformStamped__Sequence__init(&tf_message->transforms, 1);

  tf_message->transforms.data[0].header.frame_id.data = (char*)malloc(100*sizeof(char));
  char string1[] = "/panda_link0";
  memcpy(tf_message->transforms.data[0].header.frame_id.data, string1, strlen(string1) + 1);
  tf_message->transforms.data[0].header.frame_id.size = strlen(tf_message->transforms.data[0].header.frame_id.data);
  tf_message->transforms.data[0].header.frame_id.capacity = 100;

  char string2[] = "/inertial_unit";
  tf_message->transforms.data[0].child_frame_id.data =  (char*)malloc(100*sizeof(char));
  memcpy(tf_message->transforms.data[0].child_frame_id.data, string2, strlen(string2) + 1);
  tf_message->transforms.data[0].child_frame_id.size = strlen(tf_message->transforms.data[0].child_frame_id.data);
  tf_message->transforms.data[0].child_frame_id.capacity = 100;

}

void loop() {
  struct timespec tv = {0};
  clock_gettime(0, &tv);

  ////////////////////////TF_node 
  sensors.updateIMU();
  double q[4];
  sensors.euler_to_quat(q);

  tf_message->transforms.data[0].transform.rotation.x = (double) q[1];
	tf_message->transforms.data[0].transform.rotation.y = (double) q[2];
	tf_message->transforms.data[0].transform.rotation.z = (double) q[3]; 
	tf_message->transforms.data[0].transform.rotation.w = (double) q[0];
  tf_message->transforms.data[0].header.stamp.nanosec = tv.tv_nsec;
	tf_message->transforms.data[0].header.stamp.sec = tv.tv_sec;

  RCSOFTCHECK(rcl_publish(&publisher, tf_message, NULL));

  ///////////////////////target_velocity sub node
  RCSOFTCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100)));
  

  //motordriver.control_vel(0.059,0,0);
  //motordriver.doFR_Motor(HIGH,50);
  //motordriver.view_angVel();
  //motordriver.view_encoderPos();
}
