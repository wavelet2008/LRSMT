#ifndef HARDWARE_H_
#define HARDWARE_H_

#include <stdio.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/touch_sensor.h>
#include <webots/position_sensor.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>

#include "control.h"

extern struct ROBOT rockdog_03;

WbDeviceTag motor_rf_1;
WbDeviceTag motor_rf_2;
WbDeviceTag motor_lf_1;
WbDeviceTag motor_lf_2;
WbDeviceTag motor_rr_1;
WbDeviceTag motor_rr_2;
WbDeviceTag motor_lr_1;
WbDeviceTag motor_lr_2;

WbDeviceTag sensor_rf_1;
WbDeviceTag sensor_rf_2;
WbDeviceTag sensor_lf_1;
WbDeviceTag sensor_lf_2;
WbDeviceTag sensor_rr_1;
WbDeviceTag sensor_rr_2;
WbDeviceTag sensor_lr_1;
WbDeviceTag sensor_lr_2;

WbDeviceTag foot_rf;
WbDeviceTag foot_lf;
WbDeviceTag foot_rr;
WbDeviceTag foot_lr;

WbDeviceTag imu;

void device_init();                    //初始化硬件

void imu_update();                     //陀螺仪读取
void sensor_update();                  //电机角度读取
void touch_update();                   //接触开关读取
void state_update();                   //状态机和时间更新
void vel_update_leg(struct LEG*leg);   //足端速度更新
void vel_update_whole();               //整体的速度更新
void keyboard_update();                //键盘更新
void api_update();                     //外部接口数据更新

void motor_set_torque();               //电机力矩设置
//////////////////////////////////////////////////////
void device_init()                     //初始化硬件
{
  //电机初始化
  motor_rf_1 = wb_robot_get_device("motor_rf_1");
  motor_rf_2 = wb_robot_get_device("motor_rf_2");
  motor_lf_1 = wb_robot_get_device("motor_lf_1");
  motor_lf_2 = wb_robot_get_device("motor_lf_2");
  motor_rr_1 = wb_robot_get_device("motor_rr_1");
  motor_rr_2 = wb_robot_get_device("motor_rr_2");
  motor_lr_1 = wb_robot_get_device("motor_lr_1");
  motor_lr_2 = wb_robot_get_device("motor_lr_2");
  
  //传感器初始化
  sensor_rf_1 = wb_robot_get_device("sensor_rf_1");
  wb_position_sensor_enable(sensor_rf_1, TIME_STEP);
  sensor_rf_2 = wb_robot_get_device("sensor_rf_2");
  wb_position_sensor_enable(sensor_rf_2, TIME_STEP);
  
  sensor_lf_1 = wb_robot_get_device("sensor_lf_1");
  wb_position_sensor_enable(sensor_lf_1, TIME_STEP);
  sensor_lf_2 = wb_robot_get_device("sensor_lf_2");
  wb_position_sensor_enable(sensor_lf_2, TIME_STEP);
  
  sensor_rr_1 = wb_robot_get_device("sensor_rr_1");
  wb_position_sensor_enable(sensor_rr_1, TIME_STEP);
  sensor_rr_2 = wb_robot_get_device("sensor_rr_2");
  wb_position_sensor_enable(sensor_rr_2, TIME_STEP);
  
  sensor_lr_1 = wb_robot_get_device("sensor_lr_1");
  wb_position_sensor_enable(sensor_lr_1, TIME_STEP);
  sensor_lr_2 = wb_robot_get_device("sensor_lr_2");
  wb_position_sensor_enable(sensor_lr_2, TIME_STEP);
  
    //接触传感器初始化
  foot_rf = wb_robot_get_device("foot_rf");
  wb_touch_sensor_enable(foot_rf, TIME_STEP);
  foot_lf = wb_robot_get_device("foot_lf");
  wb_touch_sensor_enable(foot_lf, TIME_STEP);
  foot_rr = wb_robot_get_device("foot_rr");
  wb_touch_sensor_enable(foot_rr, TIME_STEP);
  foot_lr = wb_robot_get_device("foot_lr");
  wb_touch_sensor_enable(foot_lr, TIME_STEP);
  
  //imu初始化
  imu = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(imu, TIME_STEP);
  
  //键盘初始化
  wb_keyboard_enable(TIME_STEP);
}

void imu_update()
{
  const double* data = wb_inertial_unit_get_roll_pitch_yaw(imu);
  static struct XYZ prerpy_a = {0,0,0};
  
  rockdog_03.rpy_a.x = data[0];  
  rockdog_03.rpy_a.z = data[1];
  rockdog_03.rpy_a.y = data[2];
  
  rockdog_03.drpy_a.x = 0.1*((rockdog_03.rpy_a.x - prerpy_a.x)/0.01) + 0.9*rockdog_03.drpy_a.x;
  rockdog_03.drpy_a.z = 0.1*((rockdog_03.rpy_a.z - prerpy_a.z)/0.01) + 0.9*rockdog_03.drpy_a.z;
  rockdog_03.drpy_a.y = 0.1*((rockdog_03.rpy_a.y - prerpy_a.y)/0.01) + 0.9*rockdog_03.drpy_a.y;
  
  prerpy_a.x = rockdog_03.rpy_a.x;
  prerpy_a.y = rockdog_03.rpy_a.y;
  prerpy_a.z = rockdog_03.rpy_a.z;
  
  
}

void sensor_update()
{
  rockdog_03.leg_rf.sensor_1.pos = wb_position_sensor_get_value(sensor_rf_1);
  rockdog_03.leg_rf.sensor_2.pos = wb_position_sensor_get_value(sensor_rf_2);
  rockdog_03.leg_lf.sensor_1.pos = wb_position_sensor_get_value(sensor_lf_1);
  rockdog_03.leg_lf.sensor_2.pos = wb_position_sensor_get_value(sensor_lf_2);
  rockdog_03.leg_rb.sensor_1.pos = wb_position_sensor_get_value(sensor_rr_1);
  rockdog_03.leg_rb.sensor_2.pos = wb_position_sensor_get_value(sensor_rr_2);
  rockdog_03.leg_lb.sensor_1.pos = wb_position_sensor_get_value(sensor_lr_1);
  rockdog_03.leg_lb.sensor_2.pos = wb_position_sensor_get_value(sensor_lr_2);
  
}

void touch_update()   //接触开关读取
{
  rockdog_03.leg_rf.touch = wb_touch_sensor_get_value(foot_rf);
  rockdog_03.leg_lf.touch = wb_touch_sensor_get_value(foot_lf);
  rockdog_03.leg_rb.touch = wb_touch_sensor_get_value(foot_rr);
  rockdog_03.leg_lb.touch = wb_touch_sensor_get_value(foot_lr);
  
}

void state_update()        //状态机和时间更新
{
  if(rockdog_03.t == T)
  {
    if(rockdog_03.leg_rf.state == 1 && rockdog_03.leg_lb.state == 1)
    {
      if(rockdog_03.leg_lf.touch && rockdog_03.leg_rb.touch)
      {
        rockdog_03.leg_lf.state = 1;
        rockdog_03.leg_rb.state = 1;
        
        rockdog_03.leg_rf.state = 0;
        rockdog_03.leg_lb.state = 0;
        rockdog_03.leg_rf.curve.x_init = rockdog_03.leg_rf.point_a.x;
        rockdog_03.leg_rf.curve.y_init = rockdog_03.leg_rf.point_a.y;
        rockdog_03.leg_lb.curve.x_init = rockdog_03.leg_lb.point_a.x;
        rockdog_03.leg_lb.curve.y_init = rockdog_03.leg_lb.point_a.y;
        
        rockdog_03.t = 0;      //相位切换时候时间归零
      }
    }
    else if(rockdog_03.leg_lf.state == 1 && rockdog_03.leg_rb.state == 1)
    {
      if(rockdog_03.leg_rf.touch && rockdog_03.leg_lb.touch)
      {
        rockdog_03.leg_rf.state = 1;
        rockdog_03.leg_lb.state = 1;
        
        rockdog_03.leg_lf.state = 0;
        rockdog_03.leg_rb.state = 0;
        rockdog_03.leg_lf.curve.x_init = rockdog_03.leg_lf.point_a.x;
        rockdog_03.leg_lf.curve.y_init = rockdog_03.leg_lf.point_a.y;
        rockdog_03.leg_rb.curve.x_init = rockdog_03.leg_rb.point_a.x;
        rockdog_03.leg_rb.curve.y_init = rockdog_03.leg_rb.point_a.y;
        
        rockdog_03.t = 0;      //相位切换时候时间归零
      }
      
    }
    else
    {
      rockdog_03.t = T;
    }
  }
  else
  {
    rockdog_03.t += TIME_STEP;
  }
/**************************************************************/
  if(rockdog_03.t > 0.7*T && rockdog_03.t < T)
  {
    if(rockdog_03.leg_rf.state == 1 && rockdog_03.leg_lb.state == 1)
    {
      if(rockdog_03.leg_lf.touch && rockdog_03.leg_rb.touch)
      {
        rockdog_03.leg_lf.state = 1;
        rockdog_03.leg_rb.state = 1;
        
        rockdog_03.leg_rf.state = 0;
        rockdog_03.leg_lb.state = 0;
        rockdog_03.leg_rf.curve.x_init = rockdog_03.leg_rf.point_a.x;
        rockdog_03.leg_rf.curve.y_init = rockdog_03.leg_rf.point_a.y;
        rockdog_03.leg_lb.curve.x_init = rockdog_03.leg_lb.point_a.x;
        rockdog_03.leg_lb.curve.y_init = rockdog_03.leg_lb.point_a.y;
        
        rockdog_03.t = 0;      //相位切换时候时间归零
      }
    }
    else if(rockdog_03.leg_lf.state == 1 && rockdog_03.leg_rb.state == 1)
    {
      if(rockdog_03.leg_rf.touch && rockdog_03.leg_lb.touch)
      {
        rockdog_03.leg_rf.state = 1;
        rockdog_03.leg_lb.state = 1;
        
        rockdog_03.leg_lf.state = 0;
        rockdog_03.leg_rb.state = 0;
        rockdog_03.leg_lf.curve.x_init = rockdog_03.leg_lf.point_a.x;
        rockdog_03.leg_lf.curve.y_init = rockdog_03.leg_lf.point_a.y;
        rockdog_03.leg_rb.curve.x_init = rockdog_03.leg_rb.point_a.x;
        rockdog_03.leg_rb.curve.y_init = rockdog_03.leg_rb.point_a.y;
        
        rockdog_03.t = 0;      //相位切换时候时间归零
      }
    }
  }
/*********************************************/
  if(rockdog_03.t > T)
  {
    rockdog_03.t -= T;
  }
}

void vel_update_leg(struct LEG*leg)   //足端速度更新
{
  leg->vel_a.x = 0.5*((leg->point_a.x - leg->prepoint_a.x)/(0.001*TIME_STEP)) + 0.5*leg->vel_a.x;
  leg->vel_a.y = 0.5*((leg->point_a.y - leg->prepoint_a.y)/(0.001*TIME_STEP)) + 0.5*leg->vel_a.y;
  
  leg->prepoint_a.x = leg->point_a.x;
  leg->prepoint_a.y = leg->point_a.y;
  
}

void vel_update_whole()
{
  vel_update_leg(&rockdog_03.leg_rf);
  vel_update_leg(&rockdog_03.leg_lf);
  vel_update_leg(&rockdog_03.leg_rb);
  vel_update_leg(&rockdog_03.leg_lb);
  
  if(rockdog_03.leg_rf.state && rockdog_03.leg_lb.state)
  {
    rockdog_03.vel_a.x = -((rockdog_03.leg_rf.vel_a.x + rockdog_03.leg_lb.vel_a.x)/2);
  }
  else if(rockdog_03.leg_lf.state && rockdog_03.leg_rb.state)
  {
    rockdog_03.vel_a.x = -((rockdog_03.leg_lf.vel_a.x + rockdog_03.leg_rb.vel_a.x)/2);
  }
  
}

void keyboard_update()                //键盘更新
{
  
  switch(wb_keyboard_get_key())
  {
    case WB_KEYBOARD_UP:
    {
      //rockdog_03.vel_g.x += 0.001;    //(其中-0.16为速度补偿)
      //if(rockdog_03.vel_g.x >= (1.5-0.36)) rockdog_03.vel_g.x = (1.5-0.36);
      rockdog_03.vel_g.x = 0.5 + (-0.11);
      break;
    }
    case WB_KEYBOARD_DOWN:
    {
      //rockdog_03.vel_g.x += -0.001;   //(其中-0.16为速度补偿)
      //if(rockdog_03.vel_g.x <= (-1.5-0.36)) rockdog_03.vel_g.x = (-1.5-0.36);
      rockdog_03.vel_g.x = -0.6 + (-0.11);
      break;
    }
    default:
    {
      //if(rockdog_03.vel_g.x >= (-0.36)) rockdog_03.vel_g.x -= 0.001;
      //else if(rockdog_03.vel_g.x <= (-0.36)) rockdog_03.vel_g.x += 0.001;
      rockdog_03.vel_g.x = 0 + (-0.11);
      break;
    }
  }
}

void api_update()           //外部接口数据更新
{
  touch_update();           //接触开关读取
  
  state_update();        //状态机和时间更新
  
  imu_update();             //陀螺仪读取
  
  sensor_update();          //电机角度读取
  apc_whole(&rockdog_03);   //足端位置读取（运动学正解）
  
  vel_update_whole();          //速度更新
  
  keyboard_update();                //键盘更新
  
}


double scope(double x)
{
  if(x >= 4) return 4;
  else if(x <= -4) return -4;
  else return x;
}

void motor_set_torque()
{
  
  rockdog_03.leg_rf.motor_1.tor = scope(rockdog_03.leg_rf.motor_1.tor);
  rockdog_03.leg_rf.motor_2.tor = scope(rockdog_03.leg_rf.motor_2.tor);
  
  rockdog_03.leg_lf.motor_1.tor = scope(rockdog_03.leg_lf.motor_1.tor);
  rockdog_03.leg_lf.motor_2.tor = scope(rockdog_03.leg_lf.motor_2.tor);
  
  rockdog_03.leg_rb.motor_1.tor = scope(rockdog_03.leg_rb.motor_1.tor);
  rockdog_03.leg_rb.motor_2.tor = scope(rockdog_03.leg_rb.motor_2.tor);
  
  rockdog_03.leg_lb.motor_1.tor = scope(rockdog_03.leg_lb.motor_1.tor);
  rockdog_03.leg_lb.motor_2.tor = scope(rockdog_03.leg_lb.motor_2.tor);
  
  wb_motor_set_torque(motor_rf_1,rockdog_03.leg_rf.motor_1.tor);
  wb_motor_set_torque(motor_rf_2,rockdog_03.leg_rf.motor_2.tor);
  wb_motor_set_torque(motor_lf_1,rockdog_03.leg_lf.motor_1.tor);
  wb_motor_set_torque(motor_lf_2,rockdog_03.leg_lf.motor_2.tor);
  wb_motor_set_torque(motor_rr_1,rockdog_03.leg_rb.motor_1.tor);
  wb_motor_set_torque(motor_rr_2,rockdog_03.leg_rb.motor_2.tor);
  wb_motor_set_torque(motor_lr_1,rockdog_03.leg_lb.motor_1.tor);
  wb_motor_set_torque(motor_lr_2,rockdog_03.leg_lb.motor_2.tor);
  
  
}
 


#endif