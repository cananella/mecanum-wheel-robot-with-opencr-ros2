#ifndef MAINBOT_SENSOR_H_
#define MAINBOT_SENSOR_H_

#include <IMU.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>


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
		float* getIMU(void);
		sensor_msgs::Imu getIMU(void);
		void updateIMU(void);
		void calibrationGyro(void);
		float* getImuAngularVelocity(void);
  		float* getImuLinearAcc(void);
  		float* getImuMagnetic(void);
  		float* getOrientation(void);
		sensor_msgs::MagneticField getMag(void);
	
	
	private:
		sensor_msgs::Imu imu_msg_;
		sensor_msgs::MagneticField mag_msg_;
		cIMU imu_;
	
};






#endif
