#ifndef MAINBOT_H_
#define MAINBOT_H_

#include <stdint.h>
#include <Dynamixel2Arduino.h>
#include "mainbot_sensor.h"
#include "motordriver.h"


#define DEBUG_ENABLE 1

#if DEBUG_ENABLE
  #define DEBUG_SERIAL_BEGIN(x) SerialBT2.begin(x)
  #define DEBUG_PRINT(x) SerialBT2.print(x)
  #define DEBUG_PRINTLN(x) SerialBT2.println(x)
#else
  #define DEBUG_SERIAL_BEGIN(x) 
  #define DEBUG_PRINT(x) 
  #define DEBUG_PRINTLN(x) 
#endif

const uint8_t FIRMWARE_VER = 5; //DYNAMIXEL2Arduino v0.6.1 or higher is required.
const uint32_t INTERVAL_MS_TO_CONTROL_MOTOR = 20;
const uint32_t INTERVAL_MS_TO_UPDATE_CONTROL_ITEM = 20;


namespace mainbot{
    void begin();
    void run();
}


#endif