#include "mainbot_core_config.h"


void setup() {
  DEBUG_SERIAL.begin(57600);
  motordriver.init();
  sensors.init();
  sensors.calibrationGyro();
}

void loop() {
  
  //motordriver.control_vel(0.1,0,0);
  //motordriver.view_angVel();
}
