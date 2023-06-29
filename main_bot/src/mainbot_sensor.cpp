#include "../include/mainbot_sensor.h"

MainbotSensor::MainbotSensor(){
}


MainbotSensor::~MainbotSensor(){
}

bool MainbotSensor::init(){
	DEBUG_SERIAL.begin(57600);
	uint8_t get_error_code = 0x00;
	get_error_code = imu_.begin();
	
	if (get_error_code != 0x00)
    	DEBUG_SERIAL.println("Failed to init Sensor");
  	else
    	DEBUG_SERIAL.println("Success to init Sensor");
	
	return get_error_code;
}

void MainbotSensor::initIMU(){
	imu_.begin();
}

void MainbotSensor::updateIMU(){
	imu_.update();
}

void MainbotSensor::calibrationGyro(){
	
  uint32_t pre_time;
  uint32_t t_time;

  const uint8_t led_ros_connect = 3;

  imu_.SEN.gyro_cali_start();
  
  t_time   = millis();
  pre_time = millis();

  while(!imu_.SEN.gyro_cali_get_done())
  {
    imu_.update();

    if (millis()-pre_time > 5000)
    {
      break;
    }
    if (millis()-t_time > 100)
    {
      t_time = millis();
      setLedToggle(led_ros_connect);
    }
  }
}

float* MainbotSensor::getImuAngularVelocity(void)
{
  static float angular_vel[3];

  angular_vel[0] = imu_.SEN.gyroADC[0] * GYRO_FACTOR;
  angular_vel[1] = imu_.SEN.gyroADC[1] * GYRO_FACTOR;
  angular_vel[2] = imu_.SEN.gyroADC[2] * GYRO_FACTOR;

  return angular_vel;
}

float* MainbotSensor::getImuLinearAcc(void)
{
  static float linear_acc[3];

  linear_acc[0] = imu_.SEN.accADC[0] * ACCEL_FACTOR;
  linear_acc[1] = imu_.SEN.accADC[1] * ACCEL_FACTOR;
  linear_acc[2] = imu_.SEN.accADC[2] * ACCEL_FACTOR;

  return linear_acc;
}

float* MainbotSensor::getImuMagnetic(void)
{
  static float magnetic[3];

  magnetic[0] = imu_.SEN.magADC[0] * MAG_FACTOR;
  magnetic[1] = imu_.SEN.magADC[1] * MAG_FACTOR;
  magnetic[2] = imu_.SEN.magADC[2] * MAG_FACTOR;

  return magnetic;
}

sensor_msgs::Imu MainbotSensor::getIMU(void)
{
  imu_msg_.angular_velocity.x = imu_.SEN.gyroADC[0] * GYRO_FACTOR;
  imu_msg_.angular_velocity.y = imu_.SEN.gyroADC[1] * GYRO_FACTOR;
  imu_msg_.angular_velocity.z = imu_.SEN.gyroADC[2] * GYRO_FACTOR;
  imu_msg_.angular_velocity_covariance[0] = 0.02;
  imu_msg_.angular_velocity_covariance[1] = 0;
  imu_msg_.angular_velocity_covariance[2] = 0;
  imu_msg_.angular_velocity_covariance[3] = 0;
  imu_msg_.angular_velocity_covariance[4] = 0.02;
  imu_msg_.angular_velocity_covariance[5] = 0;
  imu_msg_.angular_velocity_covariance[6] = 0;
  imu_msg_.angular_velocity_covariance[7] = 0;
  imu_msg_.angular_velocity_covariance[8] = 0.02;

  imu_msg_.linear_acceleration.x = imu_.SEN.accADC[0] * ACCEL_FACTOR;
  imu_msg_.linear_acceleration.y = imu_.SEN.accADC[1] * ACCEL_FACTOR;
  imu_msg_.linear_acceleration.z = imu_.SEN.accADC[2] * ACCEL_FACTOR;

  imu_msg_.linear_acceleration_covariance[0] = 0.04;
  imu_msg_.linear_acceleration_covariance[1] = 0;
  imu_msg_.linear_acceleration_covariance[2] = 0;
  imu_msg_.linear_acceleration_covariance[3] = 0;
  imu_msg_.linear_acceleration_covariance[4] = 0.04;
  imu_msg_.linear_acceleration_covariance[5] = 0;
  imu_msg_.linear_acceleration_covariance[6] = 0;
  imu_msg_.linear_acceleration_covariance[7] = 0;
  imu_msg_.linear_acceleration_covariance[8] = 0.04;

  imu_msg_.orientation.w = imu_.quat[0];
  imu_msg_.orientation.x = imu_.quat[1];
  imu_msg_.orientation.y = imu_.quat[2];
  imu_msg_.orientation.z = imu_.quat[3];

  imu_msg_.orientation_covariance[0] = 0.0025;
  imu_msg_.orientation_covariance[1] = 0;
  imu_msg_.orientation_covariance[2] = 0;
  imu_msg_.orientation_covariance[3] = 0;
  imu_msg_.orientation_covariance[4] = 0.0025;
  imu_msg_.orientation_covariance[5] = 0;
  imu_msg_.orientation_covariance[6] = 0;
  imu_msg_.orientation_covariance[7] = 0;
  imu_msg_.orientation_covariance[8] = 0.0025;

  return imu_msg_;
}

float* MainbotSensor::getOrientation(void)
{
  static float orientation[4];

  orientation[0] = imu_.quat[0];
  orientation[1] = imu_.quat[1];
  orientation[2] = imu_.quat[2];
  orientation[3] = imu_.quat[3];

  return orientation;
}

sensor_msgs::MagneticField MainbotSensor::getMag(void)
{
  mag_msg_.magnetic_field.x = imu_.SEN.magADC[0] * MAG_FACTOR;
  mag_msg_.magnetic_field.y = imu_.SEN.magADC[1] * MAG_FACTOR;
  mag_msg_.magnetic_field.z = imu_.SEN.magADC[2] * MAG_FACTOR;

  mag_msg_.magnetic_field_covariance[0] = 0.0048;
  mag_msg_.magnetic_field_covariance[1] = 0;
  mag_msg_.magnetic_field_covariance[2] = 0;
  mag_msg_.magnetic_field_covariance[3] = 0;
  mag_msg_.magnetic_field_covariance[4] = 0.0048;
  mag_msg_.magnetic_field_covariance[5] = 0;
  mag_msg_.magnetic_field_covariance[6] = 0;
  mag_msg_.magnetic_field_covariance[7] = 0;
  mag_msg_.magnetic_field_covariance[8] = 0.0048;

  return mag_msg_;
}

