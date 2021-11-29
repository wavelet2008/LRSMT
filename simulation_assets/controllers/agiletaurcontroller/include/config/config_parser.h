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
#include <libconfig.h>


namespace agiletaur{
namespace control{
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
    config_t robot_config_;
    config_setting_t *setting;
    const char *str;

};

}
}


#endif