/*
 * @Author: Ryoma Liu -- ROBOLAND 
 * @Date: 2021-11-28 21:14:20 
 * @Last Modified by: Ryoma Liu
 * @Last Modified time: 2021-11-28 21:45:00
 */
#ifndef CONFIG_PARSER_HPP_
#define CONFIG_PARSER_HPP_

#include <stdio.h>
#include "iostream"
#include <stdlib.h>
#include <libconfig.h++>

using namespace std;
using namespace libconfig;

namespace agiletaur{
namespace control{

struct Base{
  double BODY_WEIGHT;
  double UPPER_LEG_LENGTH;
  double LOWER_LEG_LENGTH;
};
struct Controller{
  double KP_LF;
  double KD_LF;
  double KI_LF;

  double KP_LB;
  double KD_LB;
  double KI_LB;

  double KP_RF;
  double KD_RF;
  double KI_RF;

  double KP_RB;
  double KD_RB;
  double KI_RB;
};
struct Robot_config{
  Base robot_base;
  Controller robot_controller;
};


class ControlConfig{
  public:
    static ControlConfig& GetControlConfig() {
        static ControlConfig singleton;
        return singleton;
    }
    virtual int LoadParams();
  private:
    ControlConfig() = default;

    virtual ~ControlConfig() = default;
    Robot_config robot_config_;

};

}
}


#endif