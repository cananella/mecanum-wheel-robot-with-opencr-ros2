#ifndef MAINBOT_SENSOR_H_
#define MAINBOT_SENSOR_H_

#include <IMU.h>
#include <micro_ros_arduino.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>


#define ACCEL_FACTOR 0.000598550415   
// (ADC_Value / Scale) * 9.80665            => Range : +- 2[g]
//                                             Scale : +- 16384
#define GYRO_FACTOR 0.0010642        
// (ADC_Value/Scale) * (pi/180)             => Range : +- 2000[deg/s]
//                                             Scale : +- 16.4[deg/s]
#define MAG_FACTOR                        15e-8

#define DEBUG_SERIAL  Serial

class MainbotSensor{
	
	public:
		MainbotSensor();
		~MainbotSensor();
		
		bool init(void);
		
		void initIMU(void);
		sensor_msgs__msg__Imu getIMU(void);
		void updateIMU(void);
		void updateIMUinit(void);
		void euler_to_quat(double* q);
		void calibrationGyro(void);
		
		float* getOrientation(void);
		sensor_msgs__msg__MagneticField getMag(void);
		
	private:
		sensor_msgs__msg__Imu imu_msg_;
		sensor_msgs__msg__MagneticField mag_msg_;
		cIMU imu_;
	
		
	
};






#endif
