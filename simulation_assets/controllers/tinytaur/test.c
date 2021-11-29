//KEYBOARD_UP(上)键为前进
//KEYBOARD_DOWN(下)键为后退

#include <webots/robot.h>

#include "hardware.h"
#include "control.h"


struct ROBOT rockdog_03;

int main(int argc, char **argv) 
{
  wb_robot_init();
  device_init();     //硬件初始化
  robot_init();      //软件初始化

  while (wb_robot_step(TIME_STEP) != -1) 
  { 
    
    // api_update();            //数据收集
    // robot_control();         //算法处理
    // motor_set_torque();      //力矩输出
    
    printf("rockdog_03.rpy_a.z = %f \n",rockdog_03.rpy_a.z);
  
  }
  wb_robot_cleanup();
  return 0;
}
