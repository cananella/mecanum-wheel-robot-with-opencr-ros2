#include "../include/motordriver.h"

int FR_encoderPos;
int FL_encoderPos;
int BR_encoderPos;
int BL_encoderPos;

float FR_angelVel;
float FL_angelVel;
float BR_angelVel;
float BL_angelVel;


MainbotMotorDriver::MainbotMotorDriver(){

	ratio = 360./90./12./2.;

	FR_encoderPos = 0;
	FL_encoderPos = 0;
	BR_encoderPos = 0;
	BL_encoderPos = 0;

	FR_targetVel = 0;
	FL_targetVel = 0;
	BR_targetVel = 0;
	BL_targetVel = 0;

	l1 = MODEL_LENGTH / 2;
	l2 = MODEL_WIDTH / 2;
	R = WHEEL_RATIO ;


	FR_wight=0;
	BR_wight=0;
	FL_wight=0;
	BL_wight=0;

	FR_angelVel = 0;
	FL_angelVel = 0;
	BR_angelVel = 0;
	BL_angelVel = 0;

}

MainbotMotorDriver::~MainbotMotorDriver(){
}

void MainbotMotorDriver::init(){
	pinMode(FR_MOTOR_DRIVER_IN1,OUTPUT);
	pinMode(FR_MOTOR_DRIVER_IN2,OUTPUT);
	pinMode(FL_MOTOR_DRIVER_IN3,OUTPUT);
	pinMode(FL_MOTOR_DRIVER_IN4,OUTPUT);

	pinMode(BR_MOTOR_DRIVER_IN3,OUTPUT);
	pinMode(BR_MOTOR_DRIVER_IN4,OUTPUT);
	pinMode(BL_MOTOR_DRIVER_IN1,OUTPUT);
	pinMode(BL_MOTOR_DRIVER_IN2,OUTPUT);

	pinMode(FR_ENCODER_PIN_C1,INPUT_PULLUP);
	pinMode(FR_ENCODER_PIN_C2,INPUT_PULLUP);
	pinMode(FL_ENCODER_PIN_C1,INPUT_PULLUP);
	pinMode(FL_ENCODER_PIN_C2,INPUT_PULLUP);

	pinMode(BR_ENCODER_PIN_C1,INPUT_PULLUP);
	pinMode(BR_ENCODER_PIN_C2,INPUT_PULLUP);
	pinMode(BL_ENCODER_PIN_C1,INPUT_PULLUP);
	pinMode(BL_ENCODER_PIN_C2,INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(FR_ENCODER_PIN_C2), doEncoderFR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(FL_ENCODER_PIN_C2), doEncoderFL, CHANGE);
	attachInterrupt(digitalPinToInterrupt(BR_ENCODER_PIN_C2), doEncoderBR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(BL_ENCODER_PIN_C2), doEncoderBL, CHANGE);

}

void doEncoderFR(){
  FR_encoderPos +=(digitalRead(FR_ENCODER_PIN_C1)==digitalRead(FR_ENCODER_PIN_C2))?-1:1;
}
void doEncoderFL(){
  FL_encoderPos +=(digitalRead(FL_ENCODER_PIN_C1)==digitalRead(FL_ENCODER_PIN_C2))?1:-1;
}
void doEncoderBR(){
  BR_encoderPos +=(digitalRead(BR_ENCODER_PIN_C1)==digitalRead(BR_ENCODER_PIN_C2))?-1:1;
}
void doEncoderBL(){
  BL_encoderPos +=(digitalRead(BL_ENCODER_PIN_C1)==digitalRead(BL_ENCODER_PIN_C2))?1:-1;
}


void MainbotMotorDriver::doFR_Motor(bool dir, int vel){
  digitalWrite(FR_MOTOR_DRIVER_IN2, dir);
  analogWrite(FR_MOTOR_DRIVER_IN1,dir?(MAX_MOTOR_VEL-vel):vel);
}

void MainbotMotorDriver::doFL_Motor(bool dir, int vel){
  digitalWrite(FL_MOTOR_DRIVER_IN4, !dir);
  analogWrite(FL_MOTOR_DRIVER_IN3,!dir?(MAX_MOTOR_VEL-vel):vel);
}

void MainbotMotorDriver::doBR_Motor(bool dir, int vel){
  digitalWrite(BR_MOTOR_DRIVER_IN4, dir);
  analogWrite(BR_MOTOR_DRIVER_IN3,dir?(MAX_MOTOR_VEL-vel):vel);
}

void MainbotMotorDriver::doBL_Motor(bool dir, int vel){
  digitalWrite(BL_MOTOR_DRIVER_IN2, !dir);
  analogWrite(BL_MOTOR_DRIVER_IN1,!dir?(MAX_MOTOR_VEL-vel):vel);
}

float MainbotMotorDriver::angle_vel(int *encoderPos){
  float t1=millis();
  int deg1=*encoderPos;
  delay(8);
  float t2=millis();
  int deg2=*encoderPos;
  float angle_vel= (deg2-deg1)*ratio*1000.*DEGTORED/((t2-t1));
  return angle_vel;
  }

void MainbotMotorDriver::Stop(){
  digitalWrite(FR_MOTOR_DRIVER_IN2, LOW);
  analogWrite(FR_MOTOR_DRIVER_IN1,0);
  digitalWrite(FL_MOTOR_DRIVER_IN4, LOW);
  analogWrite(FL_MOTOR_DRIVER_IN3,0);
  digitalWrite(BR_MOTOR_DRIVER_IN4, LOW);
  analogWrite(BR_MOTOR_DRIVER_IN3,0);
  digitalWrite(BL_MOTOR_DRIVER_IN2, LOW);
  analogWrite(BL_MOTOR_DRIVER_IN1,0);
  int FR_wight=0;
  int BR_wight=0;
  int FL_wight=0;
  int BL_wight=0;
}


void MainbotMotorDriver::control_vel(double Vx, double Vy, double W){
  //vel 0.06~0.23
  if(Vx == 0.0 && Vy == 0.0 && W == 0.0) {
    this->Stop();
  }
  else{
    BR_targetVel = (Vx - Vy - (l1+l2)*W)/R;
    FR_targetVel = (Vx + Vy - (l1+l2)*W)/R;
    FL_targetVel = (Vx - Vy + (l1+l2)*W)/R;
    BL_targetVel = (Vx + Vy + (l1+l2)*W)/R;
  }
  bool FR_done_flag=false;
  bool FL_done_flag=false;
  bool BL_done_flag=false;
  bool BR_done_flag=false;
  float angVel_error = 0.400;
  float minAngVel=1.4300;

  while(!(FR_done_flag && FL_done_flag && BR_done_flag && BL_done_flag)){
    FR_angelVel = angle_vel(&FR_encoderPos);
    FL_angelVel = angle_vel(&FL_encoderPos);
    BR_angelVel = angle_vel(&BR_encoderPos);
    BL_angelVel = angle_vel(&BL_encoderPos);

    if (FR_targetVel<-minAngVel && !FR_done_flag){
      if (FR_targetVel<FR_angelVel && FR_wight<105){
        FR_wight++;
        doFR_Motor(LOW,MIN_MOTOR_VEL+FR_wight);
      }
      else if(FR_targetVel - angVel_error > FR_angelVel && FR_wight>0){
        FR_wight--;
        doFR_Motor(LOW,MIN_MOTOR_VEL+FR_wight);
      }
      else{
        doFR_Motor(LOW,MIN_MOTOR_VEL+FR_wight);
        FR_done_flag=true;
      }
    }
    else if (FR_targetVel>minAngVel && !FR_done_flag){
      if (FR_targetVel>FR_angelVel && FR_wight<105){
        FR_wight++;
        doFR_Motor(HIGH,MIN_MOTOR_VEL+FR_wight);
      }
      else if(FR_targetVel + angVel_error <FR_angelVel && FR_wight>0){
        FR_wight--;
        doFR_Motor(HIGH,MIN_MOTOR_VEL+FR_wight);
      }
      else{
        doFR_Motor(HIGH,MIN_MOTOR_VEL+FR_wight);
        FR_done_flag=true;
      }
    }
    else if(!FR_done_flag){
      doFR_Motor(LOW,0);
      FR_done_flag=true;
    }


    if (FL_targetVel < -minAngVel && !FL_done_flag) {
      if (FL_targetVel < FL_angelVel  && FL_wight<105) {
        FL_wight++;
        doFL_Motor(LOW, MIN_MOTOR_VEL + FL_wight);
      }
      else if(FL_targetVel - angVel_error > FL_angelVel && FL_wight>0){
        FL_wight--;
        doFL_Motor(LOW,MIN_MOTOR_VEL+FL_wight);
      }
      else {
        doFL_Motor(LOW, MIN_MOTOR_VEL + FL_wight);
        FL_done_flag = true;
      }
    }
    else if (FL_targetVel > minAngVel && !FL_done_flag) {
      if (FL_targetVel > FL_angelVel  && FL_wight<105) {
        FL_wight++;
        doFL_Motor(HIGH, MIN_MOTOR_VEL + FL_wight);
      }
      else if(FL_targetVel+angVel_error < FL_angelVel && FL_wight>0){
        FL_wight--;
        doFL_Motor(HIGH,MIN_MOTOR_VEL+FL_wight);
      }
      else {
        doFL_Motor(HIGH, MIN_MOTOR_VEL + FL_wight);
        FL_done_flag = true;
      }
    }
    else if(!FL_done_flag){
      doFL_Motor(LOW,0);
      FL_done_flag=true;
    }


    if (BR_targetVel<-minAngVel && !BR_done_flag){
      if (BR_targetVel<BR_angelVel && BR_wight<105 ){
        BR_wight++;
        doBR_Motor(LOW,MIN_MOTOR_VEL+BR_wight);
      }
      else if(BR_targetVel - angVel_error > BR_angelVel && BR_wight>0){
        BR_wight--;
        doBR_Motor(LOW,MIN_MOTOR_VEL+BR_wight);
      }
      else{
        doBR_Motor(LOW,MIN_MOTOR_VEL+BR_wight);
        BR_done_flag=true;
      }
    }
    else if (BR_targetVel>minAngVel && !BR_done_flag){
      if (BR_targetVel>BR_angelVel && BR_wight<105 ){
        BR_wight++;
        doBR_Motor(HIGH,MIN_MOTOR_VEL+BR_wight);
      }
      else if(BR_targetVel+angVel_error < BR_angelVel && BR_wight>0){
        BR_wight--;
        doBR_Motor(HIGH,MIN_MOTOR_VEL+BR_wight);
      }
      else{
        doBR_Motor(HIGH,MIN_MOTOR_VEL+BR_wight);
        BR_done_flag=true;
      }
    }
    else if(!BR_done_flag){
      doBR_Motor(LOW,0);
      BR_done_flag=true;
    }


    if (BL_targetVel < -minAngVel && !BL_done_flag) {
      if (BL_targetVel < BL_angelVel && BL_wight<105) {
        BL_wight++;
        doBL_Motor(LOW, MIN_MOTOR_VEL + BL_wight);
      }
      else if(BL_targetVel - angVel_error > BL_angelVel && BL_wight>0){
        BL_wight--;
        doBR_Motor(LOW,MIN_MOTOR_VEL+BL_wight);
      }
      else {
        doBL_Motor(LOW, MIN_MOTOR_VEL + BL_wight);
        BL_done_flag = true;
      }
    }
    else if (BL_targetVel > minAngVel && !BL_done_flag) {
      if (BL_targetVel > BL_angelVel  && BL_wight<105) {
        BL_wight++;
        doBL_Motor(HIGH, MIN_MOTOR_VEL + BL_wight);
      }
      else if(BL_targetVel+angVel_error < BL_angelVel && BL_wight>0){
        BL_wight--;
        doBR_Motor(HIGH,MIN_MOTOR_VEL+BL_wight);
      }
      else {
        doBL_Motor(HIGH, MIN_MOTOR_VEL + BL_wight);
        BL_done_flag = true;
      }
    }
    else if(!BL_done_flag){
      doBL_Motor(LOW,0);
      BL_done_flag=true;
    }
  }
  //DEBUG_SERIAL.println("motordone");
}

void MainbotMotorDriver::view_angVel(){
  DEBUG_SERIAL.print("          FR_angelVel : ");
  DEBUG_SERIAL.print(FR_angelVel);
  DEBUG_SERIAL.print("          FL_angelVel : ");
  DEBUG_SERIAL.print(FL_angelVel);
  DEBUG_SERIAL.print("          BR_angelVel : ");
  DEBUG_SERIAL.print(BR_angelVel);
  DEBUG_SERIAL.print("          BL_angelVel : ");
  DEBUG_SERIAL.println(BL_angelVel);
}

void MainbotMotorDriver::view_encoderPos(){
  DEBUG_SERIAL.print("FR_encoderPos : ");
  DEBUG_SERIAL.print(FR_encoderPos*ratio);
  DEBUG_SERIAL.print("   FL_encoderPos : ");
  DEBUG_SERIAL.print(FL_encoderPos*ratio);
  DEBUG_SERIAL.print("   BR_encoderPos : ");
  DEBUG_SERIAL.print(BR_encoderPos*ratio);
  DEBUG_SERIAL.print("   BL_encoderPos : ");
  DEBUG_SERIAL.println(BL_encoderPos*ratio);
}
