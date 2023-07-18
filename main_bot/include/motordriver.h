#ifndef MOTORDRIVER_H_
#define MOTORDRIVER_H_

#include <Arduino.h>

#define FR_ENCODER_PIN_C1 A5
#define FR_ENCODER_PIN_C2 2
#define FL_ENCODER_PIN_C1 A4
#define FL_ENCODER_PIN_C2 3

#define BR_ENCODER_PIN_C1 A3
#define BR_ENCODER_PIN_C2 7
#define BL_ENCODER_PIN_C1 A2
#define BL_ENCODER_PIN_C2 8

#define FR_MOTOR_DRIVER_IN1 5
#define FR_MOTOR_DRIVER_IN2 4
#define FL_MOTOR_DRIVER_IN3 6
#define FL_MOTOR_DRIVER_IN4 12

#define BR_MOTOR_DRIVER_IN3 9
#define BR_MOTOR_DRIVER_IN4 13
#define BL_MOTOR_DRIVER_IN1 10
#define BL_MOTOR_DRIVER_IN2 14

#define MODEL_LENGTH 0.27
#define MODEL_WIDTH 0.25
#define WHEEL_RATIO 0.04
#define DEGTORED 0.01745329252

#define MAX_MOTOR_VEL 225
#define MIN_MOTOR_VEL 30

#define DEBUG_SERIAL  Serial

extern int FR_encoderPos;
extern int FL_encoderPos;
extern int BR_encoderPos;
extern int BL_encoderPos;

extern float FR_angelVel;
extern float FL_angelVel;
extern float BR_angelVel;
extern float BL_angelVel;

void doEncoderFR();
void doEncoderFL();
void doEncoderBR();
void doEncoderBL();


class MainbotMotorDriver{
	public:

		MainbotMotorDriver();
		~MainbotMotorDriver();
		void init();

		void doFR_Motor(bool dir, int vel);
		void doFL_Motor(bool dir, int vel);
		void doBR_Motor(bool dir, int vel);
		void doBL_Motor(bool dir, int vel);
		float angle_vel(int *encoderPos);
		void Stop();
		void control_vel(float Vx, float Vy, float W);
		void view_angVel();
		void view_encoderPos();
			

		

	private:
		float ratio;

		float FR_targetVel;
		float FL_targetVel;
		float BR_targetVel;
		float BL_targetVel;

		float l1;
		float l2;
		float R;

		int FR_wight;
		int BR_wight;
		int FL_wight;
		int BL_wight;

};


#endif
