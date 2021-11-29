/*
 * @Author: Ryoma Liu -- ROBOLAND 
 * @Date: 2021-11-21 22:01:05 
 * @Last Modified by: Ryoma Liu
 * @Last Modified time: 2021-11-28 01:02:33
 */

#ifndef LOWERPROXY_H_
#define LOWERPROXY_H_
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/inertial_unit.h>
#include <webots/touch_sensor.h>
#include <webots/keyboard.h>
#include <webots/gps.h>
#include <webots/joystick.h>


// #include <webots/robot.hpp>
// #include <Eigen/Dense>
// #include <qpOASES.hpp>
// #include "wbInterface.h"
// #include "optimaize.h"
// #include "locomotion_header.h"


// #include "gait_math.h"
// #include "adrc.h"
// #include "convexMPC_interface.h"
// #include "common_types.h"
// #include "SolverMPC.h"
// #include "cppTypes.h"



namespace agiletaur{
namespace control{

class lowerproxy{
  public:
    static lowerproxy& GetLowerProxy() {
      static lowerproxy singleton;
      return singleton;
    }
    void Init(int);
    void PublishControlCommand();
  private:
    lowerproxy() = default;
    virtual ~lowerproxy() = default;  
    const char* motor_c[8] = { "motor_4","motor_3",
            "motor_8","motor_7",
            "motor_1","motor_2",
            "motor_5","motor_6" };
    const char* foot_c[4] = { "Foot_B","Foot_D","Foot_A","Foot_C" };
    WbDeviceTag posensor[8];
    WbDeviceTag motor[8];
    WbDeviceTag foot[4];
    WbDeviceTag IMU;
    WbDeviceTag GPS;
};

} //namespace control
} //namespace agiletaur



#endif
