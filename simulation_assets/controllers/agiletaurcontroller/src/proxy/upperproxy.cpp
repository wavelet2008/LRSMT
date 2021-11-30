/*
 * @Author: Ryoma Liu -- ROBOLAND 
 * @Date: 2021-11-21 21:58:00 
 * @Last Modified by: Ryoma Liu
 * @Last Modified time: 2021-11-28 14:38:09
 */

#include "proxy/upperproxy.h"
#include <webots/Robot.hpp>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/inertial_unit.h>
#include <webots/touch_sensor.h>
#include <webots/keyboard.h>
#include <webots/gps.h>
#include <webots/joystick.h>

/**
 * upperproxy - class to collect robot's information and trajectories from path
 * planning and decision making part. 
 * agile taur.
 */

namespace agiletaur{
namespace control{

    
    void upperproxy::Init(){

        int axis = wb_joystick_get_number_of_axes();
	    int povs = wb_joystick_get_number_of_povs();
        if (wb_joystick_is_connected()){
        int button = wb_joystick_get_pressed_button();
		printf("button = %d\n",button);

		// if (button == 11)
		// 	robotwb.ocu.key_y = 1;
		// if (button == 10)
		// 	robotwb.ocu.key_x = 1;
		// if (button == 9)
		// 	robotwb.ocu.key_b = 1;
		// if (button == 8)
		// 	robotwb.ocu.key_a = 1;
		// if (button == 4)
		// 	robotwb.ocu.key_ll = 1;
		// if (button == 5)
		// 	robotwb.ocu.key_rr = 1;
		// if (button == 0)
		// 	robotwb.ocu.key_st = 1;
		// if (button == 1)
		// 	robotwb.ocu.key_back = 1;
        // }
    }
    }

    
} //namespace control
} //namespace agiletaur