#ifndef MAINBOT_CORE_CONFIG_H_
#define MAINBOT_CORE_CONFIG_H_

#include "include/user_config.h""
#include <ros2arduino.h>
#define XRCEDDS_PORT  Serial
#define PUBLISH_FREQUENCY 2 //hz




#include "include/motordriver.h"
#include "include/mainbot_sensor.h"




MainbotMotorDriver motordriver;
MainbotSensor sensors;




#endif
