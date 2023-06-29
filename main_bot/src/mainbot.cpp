#include "../include/mainbot.h"

MainbotMotorDriver motordriver;
MainbotSensor sensors;


#define SERIAL_DXL_SLAVE Serial
const uint8_t ID_DXL_SLAVE = 200;
const uint16_t MODEL_NUM_DXL_SLAVE = 0x5000;
const float PROTOCOL_VERSION_DXL_SLAVE = 2.0;
const uint32_t HEARTBEAT_TIMEOUT_MS = 500;
static void dxl_slave_write_callback_func(uint16_t addr, uint8_t &dxl_err_code, void* arg);
static bool get_connection_state_with_ros2_node();
static void set_connection_state_with_ros2_node(bool is_connected);
static void update_connection_state_with_ros2_node();
static void update_imu(uint32_t interval_ms);
static void update_times(uint32_t interval_ms);


DYNAMIXEL::USBSerialPortHandler port_dxl_slave(SERIAL_DXL_SLAVE);
DYNAMIXEL::Slave dxl_slave(port_dxl_slave, MODEL_NUM_DXL_SLAVE);

enum ControlTableItemAddr{
  ADDR_MODEL_INFORM    = 2,
  
  ADDR_MILLIS          = 10,

  ADDR_DEBUG_MODE      = 14,  
  ADDR_CONNECT_ROS2    = 15,
  ADDR_CONNECT_MANIP   = 16,

  ADDR_DEVICE_STATUS   = 18,
  ADDR_HEARTBEAT       = 19,

  ADDR_USER_LED_1      = 20,
  ADDR_USER_LED_2      = 21,
  ADDR_USER_LED_3      = 22,
  ADDR_USER_LED_4      = 23,

  ADDR_BUTTON_1        = 26,
  ADDR_BUTTON_2        = 27,
  ADDR_BUMPER_1        = 28,
  ADDR_BUMPER_2        = 29,

  ADDR_ILLUMINATION    = 30,
  ADDR_IR              = 34,
  ADDR_SORNA           = 38,

  ADDR_BATTERY_VOLTAGE = 42,
  ADDR_BATTERY_PERCENT = 46,

  ADDR_SOUND           = 50,

  ADDR_IMU_RECALIBRATION  = 59,
  ADDR_ANGULAR_VELOCITY_X = 60,
  ADDR_ANGULAR_VELOCITY_Y = 64,
  ADDR_ANGULAR_VELOCITY_Z = 68,
  ADDR_LINEAR_ACC_X       = 72,
  ADDR_LINEAR_ACC_Y       = 76,
  ADDR_LINEAR_ACC_Z       = 80,
  ADDR_MAGNETIC_X         = 84,
  ADDR_MAGNETIC_Y         = 88,
  ADDR_MAGNETIC_Z         = 92,
  ADDR_ORIENTATION_W      = 96,
  ADDR_ORIENTATION_X      = 100,
  ADDR_ORIENTATION_Y      = 104,
  ADDR_ORIENTATION_Z      = 108,
  
  ADDR_PRESENT_CURRENT_L  = 120,
  ADDR_PRESENT_CURRENT_R  = 124,
  ADDR_PRESENT_VELOCITY_L = 128,
  ADDR_PRESENT_VELOCITY_R = 132,
  ADDR_PRESENT_POSITION_L = 136,
  ADDR_PRESENT_POSITION_R = 140,
  
  ADDR_MOTOR_CONNECT      = 148,
  ADDR_MOTOR_TORQUE       = 149,
  ADDR_CMD_VEL_LINEAR_X   = 150,
  ADDR_CMD_VEL_LINEAR_Y   = 154,
  ADDR_CMD_VEL_LINEAR_Z   = 158,
  ADDR_CMD_VEL_ANGULAR_X  = 162,
  ADDR_CMD_VEL_ANGULAR_Y  = 166,
  ADDR_CMD_VEL_ANGULAR_Z  = 170,
  ADDR_PROFILE_ACC_L      = 174,
  ADDR_PROFILE_ACC_R      = 178,

  ADDR_TORQUE_JOINT             = 199,

  ADDR_GOAL_POSITION_JOINT_1    = 200,
  ADDR_GOAL_POSITION_JOINT_2    = 204,
  ADDR_GOAL_POSITION_JOINT_3    = 208,
  ADDR_GOAL_POSITION_JOINT_4    = 212,
  ADDR_GOAL_POSITION_GRIPPER    = 216,
  ADDR_GOAL_POSITION_WR_JOINT   = 220,
  ADDR_GOAL_POSITION_WR_GRIPPER = 221,
  ADDR_GOAL_POSITION_RD         = 222,

  ADDR_PRESENT_POSITION_JOINT_1 = 224,
  ADDR_PRESENT_POSITION_JOINT_2 = 228,
  ADDR_PRESENT_POSITION_JOINT_3 = 232,
  ADDR_PRESENT_POSITION_JOINT_4 = 236,
  ADDR_PRESENT_POSITION_GRIPPER = 240,

  ADDR_PRESENT_VELOCITY_JOINT_1 = 244,
  ADDR_PRESENT_VELOCITY_JOINT_2 = 248,
  ADDR_PRESENT_VELOCITY_JOINT_3 = 252,
  ADDR_PRESENT_VELOCITY_JOINT_4 = 256,
  ADDR_PRESENT_VELOCITY_GRIPPER = 260,

  ADDR_PRESENT_CURRENT_JOINT_1  = 264,
  ADDR_PRESENT_CURRENT_JOINT_2  = 266,
  ADDR_PRESENT_CURRENT_JOINT_3  = 268,
  ADDR_PRESENT_CURRENT_JOINT_4  = 270,
  ADDR_PRESENT_CURRENT_GRIPPER  = 272,

  ADDR_PROFILE_ACC_JOINT_1      = 284,
  ADDR_PROFILE_ACC_JOINT_2      = 288,
  ADDR_PROFILE_ACC_JOINT_3      = 292,
  ADDR_PROFILE_ACC_JOINT_4      = 296,
  ADDR_PROFILE_ACC_GRIPPER      = 300,
  ADDR_PROFILE_ACC_WR_JOINT     = 304,
  ADDR_PROFILE_ACC_WR_GRIPPER   = 305,
  ADDR_PROFILE_ACC_RD           = 306,

  ADDR_PROFILE_VEL_JOINT_1      = 308,
  ADDR_PROFILE_VEL_JOINT_2      = 312,
  ADDR_PROFILE_VEL_JOINT_3      = 316,
  ADDR_PROFILE_VEL_JOINT_4      = 320,
  ADDR_PROFILE_VEL_GRIPPER      = 324,
  ADDR_PROFILE_VEL_WR_JOINT     = 328,
  ADDR_PROFILE_VEL_WR_GRIPPER   = 329,
  ADDR_PROFILE_VEL_RD           = 330,

  ADDR_GOAL_CURRENT_JOINT_1     = 332,
  ADDR_GOAL_CURRENT_JOINT_2     = 334,
  ADDR_GOAL_CURRENT_JOINT_3     = 336,
  ADDR_GOAL_CURRENT_JOINT_4     = 338,
  ADDR_GOAL_CURRENT_GRIPPER     = 340,  
  ADDR_GOAL_CURRENT_WR_JOINT    = 342,
  ADDR_GOAL_CURRENT_WR_GRIPPER  = 343,
  ADDR_GOAL_CURRENT_RD          = 344,

};


typedef struct ControlItemVariables{
  uint32_t model_inform;

  uint32_t dev_time_millis;
  uint32_t dev_time_micros;

  int8_t device_status;
  uint8_t heart_beat;
  bool debug_mode;
  bool is_connect_ros2_node;
  bool is_connect_motors;
  bool is_connect_manipulator;

  bool user_led[4];
  bool push_button[2];
  bool bumper[2];

  uint16_t illumination;
  uint32_t ir_sensor;
  float sornar;

  uint32_t bat_voltage_x100;
  uint32_t bat_percent_x100;

  uint8_t buzzer_sound;

  bool imu_recalibration;
  float angular_vel[3];
  float linear_acc[3];
  float magnetic[3];
  float orientation[4];

  int32_t present_position[MortorLocation::MOTOR_NUM_MAX];
  int32_t present_velocity[MortorLocation::MOTOR_NUM_MAX];
  int32_t present_current[MortorLocation::MOTOR_NUM_MAX];

  bool motor_torque_enable_state;
  int32_t cmd_vel_linear[3];
  int32_t cmd_vel_angular[3];
  uint32_t profile_acceleration[MortorLocation::MOTOR_NUM_MAX];

  bool joint_torque_enable_state;
  joint_position_info_t joint_goal_position;  
  joint_position_info_t joint_present_position;
  joint_velocity_info_t joint_present_velocity;
  joint_current_info_t joint_present_current;
  joint_accel_info_t joint_profile_acc;
  joint_accel_info_t joint_profile_vel;
  joint_current_info_t joint_goal_current;

  bool joint_goal_position_wr_joint;
  bool joint_goal_position_wr_gripper;
  bool joint_goal_position_rd;

  bool joint_profile_acc_wr_joint;
  bool joint_profile_acc_wr_gripper;
  bool joint_profile_acc_rd;

  bool joint_profile_vel_wr_joint;
  bool joint_profile_vel_wr_gripper;
  bool joint_profile_vel_rd;

  bool joint_goal_current_wr_joint;
  bool joint_goal_current_wr_gripper;
  bool joint_goal_current_rd;

}ControlItemVariables;

static ControlItemVariables control_items;


void TurtleBot3Core::begin(const char* model_name)
{
  uint16_t model_motor_rpm;

  if(strcmp(model_name, "Burger") == 0 || strcmp(model_name, "burger") == 0){
    p_tb3_model_info = &burger_info;
    model_motor_rpm = 61;
  }else if(strcmp(model_name, "Waffle") == 0 || strcmp(model_name, "waffle") == 0){
    p_tb3_model_info = &waffle_info;
    model_motor_rpm = 77;
  }else if(strcmp(model_name, "Waffle_OpenManipulator") == 0){
    p_tb3_model_info = &waffle_with_manipulator_info;
    model_motor_rpm = 77;
  }else{
    p_tb3_model_info = &burger_info;
    model_motor_rpm = 61;
  }

  max_linear_velocity = p_tb3_model_info->wheel_radius*2*PI*model_motor_rpm/60;
  min_linear_velocity = -max_linear_velocity;
  max_angular_velocity = max_linear_velocity/p_tb3_model_info->turning_radius;
  min_angular_velocity = -max_angular_velocity;

  bool ret; (void)ret;
  DEBUG_SERIAL_BEGIN(57600);
  DEBUG_PRINTLN(" ");
  DEBUG_PRINTLN("Version : V221004R1");
  DEBUG_PRINTLN("Begin Start...");

  // Setting for Dynamixel motors
  ret = motor_driver.init();
  DEBUG_PRINTLN(ret==true?"Motor driver setup completed.":"Motor driver setup failed.");
  // Setting for IMU
  ret = sensors.init();
  DEBUG_PRINTLN(ret==true?"Sensors setup completed.":"Sensors setup failed.");
  // Init diagnosis
  ret = diagnosis.init();
  DEBUG_PRINTLN(ret==true?"Diagnosis setup completed.":"Diagnosis setup failed.");
  // Setting for ROBOTIS RC100 remote controller and cmd_vel
  ret = controllers.init(max_linear_velocity, max_angular_velocity);
  DEBUG_PRINTLN(ret==true?"RC100 Controller setup completed.":"RC100 Controller setup failed.");

  if (p_tb3_model_info->has_manipulator == true)
  {    
    ret = manipulator_driver.init();
    DEBUG_PRINTLN(ret==true?"Manipulator driver setup completed.":"Manipulator driver setup failed.");
  }

  DEBUG_PRINT("Dynamixel2Arduino Item Max : ");
  DEBUG_PRINTLN(CONTROL_ITEM_MAX);

  control_items.debug_mode = false;
  control_items.is_connect_ros2_node = false;
  control_items.is_connect_manipulator = false;  

  // Port begin
  dxl_slave.begin();
  // Init DXL Slave function
  dxl_slave.setPortProtocolVersion(PROTOCOL_VERSION_DXL_SLAVE);
  dxl_slave.setFirmwareVersion(FIRMWARE_VER);
  dxl_slave.setID(ID_DXL_SLAVE);

  /* Add control items for Slave */
  // Items for model information of device
  control_items.model_inform = p_tb3_model_info->model_info;
  dxl_slave.addControlItem(ADDR_MODEL_INFORM, control_items.model_inform);
  // Items for Timer of device
  dxl_slave.addControlItem(ADDR_MILLIS, control_items.dev_time_millis);

  // Items to debug mode
  dxl_slave.addControlItem(ADDR_DEBUG_MODE, control_items.debug_mode);
  // Items to connect ros2
  dxl_slave.addControlItem(ADDR_CONNECT_ROS2, control_items.is_connect_ros2_node);
  // Items to connect manipulator
  dxl_slave.addControlItem(ADDR_CONNECT_MANIP, control_items.is_connect_manipulator);

  // Items to inform device status
  dxl_slave.addControlItem(ADDR_DEVICE_STATUS, control_items.device_status);
  // Items to check connection state with node
  dxl_slave.addControlItem(ADDR_HEARTBEAT, control_items.heart_beat);
  // Items for GPIO
  dxl_slave.addControlItem(ADDR_USER_LED_1, control_items.user_led[0]);
  dxl_slave.addControlItem(ADDR_USER_LED_2, control_items.user_led[1]);
  dxl_slave.addControlItem(ADDR_USER_LED_3, control_items.user_led[2]);
  dxl_slave.addControlItem(ADDR_USER_LED_4, control_items.user_led[3]);
  dxl_slave.addControlItem(ADDR_BUTTON_1, control_items.push_button[0]);
  dxl_slave.addControlItem(ADDR_BUTTON_2, control_items.push_button[1]);
  dxl_slave.addControlItem(ADDR_BUMPER_1, control_items.bumper[0]);
  dxl_slave.addControlItem(ADDR_BUMPER_2, control_items.bumper[1]);
  // Items for Analog sensors
  dxl_slave.addControlItem(ADDR_ILLUMINATION, control_items.illumination);
  dxl_slave.addControlItem(ADDR_IR, control_items.ir_sensor);
  dxl_slave.addControlItem(ADDR_SORNA, control_items.sornar);
  // Items for Battery
  dxl_slave.addControlItem(ADDR_BATTERY_VOLTAGE, control_items.bat_voltage_x100);
  dxl_slave.addControlItem(ADDR_BATTERY_PERCENT, control_items.bat_percent_x100);
  // Items for Buzzer
  dxl_slave.addControlItem(ADDR_SOUND, control_items.buzzer_sound);
  // Items for IMU
  dxl_slave.addControlItem(ADDR_IMU_RECALIBRATION, control_items.imu_recalibration);
  dxl_slave.addControlItem(ADDR_ANGULAR_VELOCITY_X, control_items.angular_vel[0]);
  dxl_slave.addControlItem(ADDR_ANGULAR_VELOCITY_Y, control_items.angular_vel[1]);
  dxl_slave.addControlItem(ADDR_ANGULAR_VELOCITY_Z, control_items.angular_vel[2]);
  dxl_slave.addControlItem(ADDR_LINEAR_ACC_X, control_items.linear_acc[0]);
  dxl_slave.addControlItem(ADDR_LINEAR_ACC_Y, control_items.linear_acc[1]);
  dxl_slave.addControlItem(ADDR_LINEAR_ACC_Z, control_items.linear_acc[2]);
  dxl_slave.addControlItem(ADDR_MAGNETIC_X, control_items.magnetic[0]);
  dxl_slave.addControlItem(ADDR_MAGNETIC_Y, control_items.magnetic[1]);
  dxl_slave.addControlItem(ADDR_MAGNETIC_Z, control_items.magnetic[2]);
  dxl_slave.addControlItem(ADDR_ORIENTATION_W, control_items.orientation[0]);
  dxl_slave.addControlItem(ADDR_ORIENTATION_X, control_items.orientation[1]);
  dxl_slave.addControlItem(ADDR_ORIENTATION_Y, control_items.orientation[2]);
  dxl_slave.addControlItem(ADDR_ORIENTATION_Z, control_items.orientation[3]);
  // Items to check status of motors
  dxl_slave.addControlItem(ADDR_PRESENT_POSITION_L, control_items.present_position[MortorLocation::LEFT]);
  dxl_slave.addControlItem(ADDR_PRESENT_POSITION_R, control_items.present_position[MortorLocation::RIGHT]);
  dxl_slave.addControlItem(ADDR_PRESENT_VELOCITY_L, control_items.present_velocity[MortorLocation::LEFT]);
  dxl_slave.addControlItem(ADDR_PRESENT_VELOCITY_R, control_items.present_velocity[MortorLocation::RIGHT]);
  dxl_slave.addControlItem(ADDR_PRESENT_CURRENT_L, control_items.present_current[MortorLocation::LEFT]);
  dxl_slave.addControlItem(ADDR_PRESENT_CURRENT_R, control_items.present_current[MortorLocation::RIGHT]);
  // Items to control motors
  dxl_slave.addControlItem(ADDR_MOTOR_CONNECT, control_items.is_connect_motors);
  dxl_slave.addControlItem(ADDR_MOTOR_TORQUE, control_items.motor_torque_enable_state);
  dxl_slave.addControlItem(ADDR_CMD_VEL_LINEAR_X, control_items.cmd_vel_linear[0]);
  dxl_slave.addControlItem(ADDR_CMD_VEL_LINEAR_Y, control_items.cmd_vel_linear[1]);
  dxl_slave.addControlItem(ADDR_CMD_VEL_LINEAR_Z, control_items.cmd_vel_linear[2]);
  dxl_slave.addControlItem(ADDR_CMD_VEL_ANGULAR_X, control_items.cmd_vel_angular[0]);
  dxl_slave.addControlItem(ADDR_CMD_VEL_ANGULAR_Y, control_items.cmd_vel_angular[1]);
  dxl_slave.addControlItem(ADDR_CMD_VEL_ANGULAR_Z, control_items.cmd_vel_angular[2]);  
  dxl_slave.addControlItem(ADDR_PROFILE_ACC_L, control_items.profile_acceleration[MortorLocation::LEFT]);
  dxl_slave.addControlItem(ADDR_PROFILE_ACC_R, control_items.profile_acceleration[MortorLocation::RIGHT]);

  // Set user callback function for processing write command from master.
  dxl_slave.setWriteCallbackFunc(dxl_slave_write_callback_func);

  // Check connection state with motors.
  if(motor_driver.is_connected() == true){
    motor_driver.set_torque(true);
    control_items.device_status = STATUS_RUNNING;
    set_connection_state_with_motors(true);
    DEBUG_PRINTLN("Wheel motors are connected");
  }else{
    control_items.device_status = STATUS_NOT_CONNECTED_MOTORS;
    set_connection_state_with_motors(false);
    DEBUG_PRINTLN("Can't communicate with the motor!");
    DEBUG_PRINTLN("  Please check the connection to the motor and the power supply.");
    DEBUG_PRINTLN();
  } 
  control_items.is_connect_motors = get_connection_state_with_motors();  

  if (p_tb3_model_info->has_manipulator == true) {
    // Check connection state with joints.
    if(manipulator_driver.is_connected() == true){
      manipulator_driver.set_torque(true);    
      control_items.is_connect_manipulator = true;
      set_connection_state_with_joints(true);
      DEBUG_PRINTLN("Joint motors are connected");      
    }else{
      control_items.is_connect_manipulator = false;
      set_connection_state_with_joints(false);
      DEBUG_PRINTLN("Can't communicate with the joint!");
      DEBUG_PRINTLN("  Please check the connection to the joint motor and the power supply.");
      DEBUG_PRINTLN();
    } 
  }

  // Init IMU 
  sensors.initIMU();
  sensors.calibrationGyro();

  //To indicate that the initialization is complete.
  sensors.makeMelody(1); 

  DEBUG_PRINTLN("Begin End...");
}


void TurtleBot3Core::run()
{
  static uint32_t pre_time_to_control_motor;

  // Check connection state with ROS2 node
  update_connection_state_with_ros2_node();

  /* For diagnosis */
  // Show LED status
  diagnosis.showLedStatus(get_connection_state_with_ros2_node());
  // Update Voltage
  diagnosis.updateVoltageCheck(true);
  // Check push button pressed for simple test drive
  test_motors_with_buttons(diagnosis.getButtonPress(3000));

  /* For sensing and run buzzer */
  // Update the IMU unit
  sensors.updateIMU();
  // Update sonar data
  // TODO: sensors.updateSonar(t);
  // Run buzzer if there is still melody to play.
  sensors.onMelody();

  /* For getting command from rc100 */
  // Receive data from RC100 
  controllers.getRCdata(goal_velocity_from_rc100);

  /* For processing DYNAMIXEL slave function */
  // Update control table of OpenCR to communicate with ROS2 node
  update_imu(INTERVAL_MS_TO_UPDATE_CONTROL_ITEM);
  update_times(INTERVAL_MS_TO_UPDATE_CONTROL_ITEM);
  update_gpios(INTERVAL_MS_TO_UPDATE_CONTROL_ITEM);
  update_motor_status(INTERVAL_MS_TO_UPDATE_CONTROL_ITEM);
  update_battery_status(INTERVAL_MS_TO_UPDATE_CONTROL_ITEM);
  update_analog_sensors(INTERVAL_MS_TO_UPDATE_CONTROL_ITEM);
  update_joint_status(INTERVAL_MS_TO_UPDATE_CONTROL_ITEM);

  // Packet processing with ROS2 Node.
  dxl_slave.processPacket();

  /* For controlling DYNAMIXEL motors (Wheels) */  
  if (millis()-pre_time_to_control_motor >= INTERVAL_MS_TO_CONTROL_MOTOR)
  {
    pre_time_to_control_motor = millis();
    if(get_connection_state_with_ros2_node() == false){
      memset(goal_velocity_from_cmd, 0, sizeof(goal_velocity_from_cmd));
    }
    update_goal_velocity_from_3values();
    if(get_connection_state_with_motors() == true){
      motor_driver.control_motors(p_tb3_model_info->wheel_separation, goal_velocity[VelocityType::LINEAR], goal_velocity[VelocityType::ANGULAR]);
    }
  }  
}


void update_times(uint32_t interval_ms)
{
  static uint32_t pre_time = 0;

  if(millis() - pre_time >= interval_ms){
    pre_time = millis();

    control_items.dev_time_millis = millis();
    control_items.dev_time_micros = micros();
  } 
}


void update_imu(uint32_t interval_ms)
{
  static uint32_t pre_time = 0;
  float* p_imu_data;

  if(millis() - pre_time >= interval_ms){
    pre_time = millis();

    p_imu_data = sensors.getImuAngularVelocity();
    memcpy(control_items.angular_vel, p_imu_data, sizeof(control_items.angular_vel));

    p_imu_data = sensors.getImuLinearAcc();
    memcpy(control_items.linear_acc, p_imu_data, sizeof(control_items.linear_acc));

    p_imu_data = sensors.getImuMagnetic();
    memcpy(control_items.magnetic, p_imu_data, sizeof(control_items.magnetic));

    p_imu_data = sensors.getOrientation();
    memcpy(control_items.orientation, p_imu_data, sizeof(control_items.orientation));
  }  
}


static void dxl_slave_write_callback_func(uint16_t item_addr, uint8_t &dxl_err_code, void* arg)
{
  (void)arg;

  switch(item_addr)
  {
    case ADDR_MODEL_INFORM:
      control_items.model_inform = p_tb3_model_info->model_info;
      dxl_err_code = DXL_ERR_ACCESS;
      break;

    case ADDR_DEBUG_MODE:
      if (control_items.debug_mode == true)
        DEBUG_PRINTLN("Debug Mode : Enabled");
      else
        DEBUG_PRINTLN("Debug Mode : Disabled");
      break;

    case ADDR_SOUND:
      sensors.makeMelody(control_items.buzzer_sound);
      break;

    case ADDR_IMU_RECALIBRATION:
      if(control_items.imu_recalibration == true){
        sensors.calibrationGyro();
        control_items.imu_recalibration = false;
      }
      break;

    case ADDR_MOTOR_TORQUE:
      if(get_connection_state_with_motors() == true)
        motor_driver.set_torque(control_items.motor_torque_enable_state);
      break;

    case ADDR_CMD_VEL_LINEAR_X:
      goal_velocity_from_cmd[VelocityType::LINEAR] = constrain((float)(control_items.cmd_vel_linear[0]*0.01f), min_linear_velocity, max_linear_velocity);
      break;

    case ADDR_CMD_VEL_ANGULAR_Z:
      goal_velocity_from_cmd[VelocityType::ANGULAR] = constrain((float)(control_items.cmd_vel_angular[2]*0.01f), min_angular_velocity, max_angular_velocity);
      break;            

    case ADDR_PROFILE_ACC_L:
    case ADDR_PROFILE_ACC_R:
      if(get_connection_state_with_motors() == true)
        motor_driver.write_profile_acceleration(control_items.profile_acceleration[MortorLocation::LEFT], control_items.profile_acceleration[MortorLocation::RIGHT]);
      break;

    case ADDR_TORQUE_JOINT:
      manipulator_driver.set_torque(control_items.joint_torque_enable_state);
      break;

    // ADDR_GOAL_POSITION
    //
    case ADDR_GOAL_POSITION_WR_JOINT:
      if (get_connection_state_with_ros2_node() == true && control_items.joint_goal_position_wr_joint == true) {
        manipulator_driver.write_goal_position_joint(control_items.joint_goal_position);
      }
      control_items.joint_goal_position_wr_joint = false;
      break;

    case ADDR_GOAL_POSITION_WR_GRIPPER:
      if (get_connection_state_with_ros2_node() == true && control_items.joint_goal_position_wr_gripper == true) {
        manipulator_driver.write_goal_position_gripper(control_items.joint_goal_position);
      }
      control_items.joint_goal_position_wr_gripper = false;
      break;

    case ADDR_GOAL_POSITION_RD:
      if (control_items.joint_goal_position_rd == true) {
        manipulator_driver.read_goal_position(control_items.joint_goal_position);
      }
      control_items.joint_goal_position_rd = false;
      break;

    // ADDR_PROFILE_ACC
    //
    case ADDR_PROFILE_ACC_WR_JOINT:
      if (get_connection_state_with_ros2_node() == true && control_items.joint_profile_acc_wr_joint == true) {
        manipulator_driver.write_profile_acceleration_joint(control_items.joint_profile_acc);
      }
      control_items.joint_profile_acc_wr_joint = false;
      break;      

    case ADDR_PROFILE_ACC_WR_GRIPPER:
      if (get_connection_state_with_ros2_node() == true && control_items.joint_profile_acc_wr_gripper == true) {
        manipulator_driver.write_profile_acceleration_gripper(control_items.joint_profile_acc);
      }
      control_items.joint_profile_acc_wr_joint = false;
      break;      

    case ADDR_PROFILE_ACC_RD:
      if (control_items.joint_profile_acc_rd == true) {
        manipulator_driver.read_profile_acceleration(control_items.joint_profile_acc);
      }
      control_items.joint_profile_acc_rd = false;
      break;     

    // ADDR_PROFILE_VEL
    //
    case ADDR_PROFILE_VEL_WR_JOINT:
      if (get_connection_state_with_ros2_node() == true && control_items.joint_profile_vel_wr_joint == true) {
        manipulator_driver.write_profile_velocity_joint(control_items.joint_profile_vel);
      }
      control_items.joint_profile_vel_wr_joint = false;
      break;      

    case ADDR_PROFILE_VEL_WR_GRIPPER:
      if (get_connection_state_with_ros2_node() == true && control_items.joint_profile_vel_wr_gripper == true) {
        manipulator_driver.write_profile_velocity_gripper(control_items.joint_profile_vel);
      }
      control_items.joint_profile_vel_wr_gripper = false;
      break;   

    case ADDR_PROFILE_VEL_RD:
      if (control_items.joint_profile_vel_rd == true) {
        manipulator_driver.read_profile_velocity(control_items.joint_profile_vel);
      }
      control_items.joint_profile_vel_rd = false;
      break;      

    // ADDR_GOAL_CURRENT
    //
    case ADDR_GOAL_CURRENT_WR_JOINT:
      if (get_connection_state_with_ros2_node() == true && control_items.joint_goal_current_wr_joint == true) {
        manipulator_driver.write_goal_current_joint(control_items.joint_goal_current);
      }
      control_items.joint_goal_current_wr_joint = false;
      break;      

    case ADDR_GOAL_CURRENT_WR_GRIPPER:
      if (get_connection_state_with_ros2_node() == true && control_items.joint_goal_current_wr_gripper == true) {
        manipulator_driver.write_goal_current_gripper(control_items.joint_goal_current);
      }
      control_items.joint_goal_current_wr_gripper = false;
      break;   

    case ADDR_GOAL_CURRENT_RD:
      if (control_items.joint_goal_current_rd == true) {
        manipulator_driver.read_goal_current(control_items.joint_goal_current);
      }
      control_items.joint_goal_current_rd = false;
      break;        
  }
}

static bool connection_state_with_ros2_node = false;

static bool get_connection_state_with_ros2_node()
{
  return connection_state_with_ros2_node;
}

static void set_connection_state_with_ros2_node(bool is_connected)
{
  connection_state_with_ros2_node = is_connected;
}

void update_connection_state_with_ros2_node()
{
  static uint32_t pre_time;
  static uint8_t pre_data;
  static bool pre_state;

  //To wait for IMU Calibration
  if(pre_state != get_connection_state_with_ros2_node()){
    pre_state = get_connection_state_with_ros2_node();
    pre_time = millis();
    return;
  }

  if(pre_data != control_items.heart_beat || control_items.debug_mode == true){
    pre_time = millis();
    pre_data = control_items.heart_beat;
    set_connection_state_with_ros2_node(true);
  }else{
    if(millis()-pre_time >= HEARTBEAT_TIMEOUT_MS){
      pre_time = millis();
      set_connection_state_with_ros2_node(false);
    }
  }

  control_items.is_connect_ros2_node = get_connection_state_with_ros2_node();
}

