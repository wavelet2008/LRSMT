#include <stdio.h>
#include <stdarg.h> 
#include <stdlib.h >
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/inertial_unit.h>
#include <webots/touch_sensor.h>
#include <webots/keyboard.h>
#include <webots/gps.h>
#include <webots/joystick.h>


extern WbDeviceTag posensor[8];
extern WbDeviceTag motor[8];
extern WbDeviceTag foot[4];
extern WbDeviceTag IMU;
extern WbDeviceTag GPS;

#define TIME_STEP   5//ms

void webots_device_init(void);
