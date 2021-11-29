/*
 * @Author: Ryoma Liu -- ROBOLAND 
 * @Date: 2021-11-21 21:58:00 
 * @Last Modified by: Ryoma Liu
 * @Last Modified time: 2021-11-28 14:38:06
 */

#include "proxy/lowerproxy.h"

/**
 * lowerproxy - class to publish control command to webots agiletaur or real 
 * agile taur.
 */

namespace agiletaur{
namespace control{
    void lowerproxy::Init(int TIME_STEP){
        IMU = wb_robot_get_device("imu");
        wb_inertial_unit_enable(IMU, TIME_STEP);    
        GPS = wb_robot_get_device("gps");
	    wb_gps_enable(GPS, TIME_STEP);
        for (u_int8_t i = 0; i < 8; i++) {
            motor[i] = wb_robot_get_device(motor_c[i]);
            posensor[i] = wb_motor_get_position_sensor(motor[i]);
            wb_position_sensor_enable(posensor[i], TIME_STEP);
            wb_motor_enable_torque_feedback(motor[i], TIME_STEP);
	    }
        for (u_int8_t i = 0; i < 4; i++) {
            foot[i] = wb_robot_get_device(foot_c[i]);
            wb_touch_sensor_enable(foot[i], TIME_STEP);
            wb_keyboard_enable(TIME_STEP);
            wb_joystick_enable(TIME_STEP);
	    }
        std::cout<<"init the robot done" <<std::endl;
    }

    void lowerproxy::PublishControlCommand(){
        std::cout<<"publish control command"<< std::endl;
    }

    
} //namespace control
} //namespace agiletaur



