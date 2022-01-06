#ifndef CONTROL_H
#define CONTROL_H

#include <stdio.h>
#include <math.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/touch_sensor.h>
#include <webots/position_sensor.h>
#include <webots/keyboard.h>

#include "hardware.h"

//最小时间单位
#define TIME_STEP 1

//圆周率
#define PI 3.1415926

//大腿长度
#define L1 0.1

//小腿长度
#define L2 0.2

//支撑相和摆动相的周期
#define T 200 

//机器人的重量
#define M 9.2

//重力加速度
#define G 9.81

//机器人的宽
#define WIDE 0.17

//机器人的长度
#define LENGTH 0.377

extern struct ROBOT rockdog_03;

struct Motor
{
    double pos;
    double vel;
    double acc;
    double tor;
    
};

struct Sensor
{
    double pos;
    double vel;
    double acc;
    double tor;
    
};

struct XYZ
{
    double x;
    double y;
    double z;
};

struct CURVE
{
  double h_pos; //期望抬腿高度
  double x_init;
  double y_init;        //起始点
  double x_lay;
  double y_lay;        //落足点
};

struct LEG
{
  struct Motor motor_1;
  struct Motor motor_2;
  
  struct Sensor sensor_1;
  struct Sensor sensor_2;
  
  bool touch;     //足端触地状态(1支撑相，0摆动相)
  int state;      //腿部状态  1(支撑相) 0(摆动相)
  
  double t;       //支撑相，摆动相的时间
  
  struct CURVE curve;
  
  struct XYZ point_a;       //足端实际位置
  struct XYZ prepoint_a;    //足端过去位置
  struct XYZ point_g;       //足端目标位置
  
  struct XYZ vel_a;         //实际足端速度
  
  struct XYZ force_g;       //期望力
  
};

struct ROBOT
{
  struct LEG leg_rf;
  struct LEG leg_lf;
  struct LEG leg_rb;
  struct LEG leg_lb;
  
  struct XYZ rpy_a;      //陀螺仪实际角度(xzy)
  struct XYZ drpy_a;     //陀螺仪实际角速度(xzy_d)
  struct XYZ rpy_g;      //机器人目标角度（xzy）
  struct XYZ drpy_g;     //陀螺仪目标角速度(xzy_d)
  
  struct XYZ vel_a;      //机器人实际速度
  struct XYZ vel_g;      //机器人期望速度
  
  int t;                 //机器人整体时间
  
  double h_g;            //目标高度 0.4
  double kh;             //机器人的高度比例系数 8000
  double kdh;            //机器人的高度微分系数 200
  
  double kr;             //机器人的roll角度比例系数
  double kdr;            //机器人的roll角度微分系数
  
  double kp;             //机器人的pitch角度比例系数
  double kdp;            //机器人的pitch角度微分系数
  
  double ky;             //机器人的yaw角速度比例系数
  double kdy;            //机器人的yaw角速度微分系数
  
  double pitch;          //自身俯仰角
  double dpitch;         //自身俯仰角的微分
  
  double k_a;            //摆动相加速度比例系数
  
};

void leg_init(struct LEG*leg,float t,int state);
void robot_init();

void apc(struct LEG*leg);                   //运动学正解
void apc_whole(struct ROBOT*robot);

void jacobian(struct LEG*leg);              //雅可比矩阵变化
void jacobian_whole(struct ROBOT*robot);    //雅可比矩阵变化

void curve_leg(struct LEG*leg);             //轨迹规划
void curve_whole(struct ROBOT*robot);       //轨迹规划

void robot_control();                       //机器人算法处理
void tor_map();                             //力矩映射

///////////////////////////////////////////////////////////////////////////////////////

void leg_init(struct LEG*leg,float t,int state)
{
  leg->t = t;                     //腿的时间
  
  leg->state = state;             //（1为支撑相）（0为摆动相）
  
  leg->curve.h_pos = -0.11;        //轨迹的参数
  leg->curve.x_init = 0;
  leg->curve.y_init = -0.2;
  leg->curve.x_lay = 0;
  leg->curve.y_lay = -0.28;
  
  leg->point_g.y = -0.2;
  leg->point_g.x = 0;              //足端目标位置
  leg->point_a.x = 0;
  leg->point_a.y = -0.17;          //足端实际位置
  leg->prepoint_a.x = 0;
  leg->prepoint_a.y = -0.17;       //足端过去位置
  
  leg->vel_a.x = 0;
  leg->vel_a.y = 0;                //足端实际速度
  
  leg->force_g.x = 0;              //足端目标力
  leg->force_g.y = 0;
}

void robot_init()
{
  rockdog_03.t = 0;                //机器人时间
  
  rockdog_03.rpy_g.x = 0;
  rockdog_03.rpy_g.y = 0;
  rockdog_03.rpy_g.z = 0;          //机器人期望rpy角度
  rockdog_03.drpy_g.x = 0;
  rockdog_03.drpy_g.y = 0;
  rockdog_03.drpy_g.z = 0;         //机器人期望rpy角速度
  
  rockdog_03.kr = 1000;            //机器人的roll角度比例系数
  rockdog_03.kdr = -500;           //机器人的roll角度微分系数
  
  rockdog_03.kp = 1000;            //机器人的pitch角度比例系数500
  rockdog_03.kdp = -300;           //机器人的pitch角度微分系数-150
  
  rockdog_03.ky = 0;               //机器人的yaw角速度比例系数1700
  rockdog_03.kdy = 0;              //机器人的yaw角速度微分系数
  
  rockdog_03.h_g = 0.2;            //机器人期望高度
  rockdog_03.kh = 10000;           //机器人的高度比例系数8000
  rockdog_03.kdh = -400;           //机器人的高度微分系数-200
  
  rockdog_03.vel_g.x = (-0.16);      //机器人期望速度  (其中-0.16为速度补偿)
  rockdog_03.vel_g.y = 0;
  rockdog_03.vel_g.z = 0;
  rockdog_03.vel_a.x = 0;          //机器人实际速度
  rockdog_03.vel_a.y = 0;
  rockdog_03.vel_a.z = 0;
  
  rockdog_03.pitch = 0;
  rockdog_03.dpitch = 0;
  
  rockdog_03.k_a = 0.04;//0.02;          //加速度比例系数(摆动相参数)
  ///////////////////////////////////////////////////////////////////////////
  leg_init(&rockdog_03.leg_rf,0,1);
  leg_init(&rockdog_03.leg_lf,0,0);
  leg_init(&rockdog_03.leg_rb,0,0);
  leg_init(&rockdog_03.leg_lb,0,1);
  
}

void apc(struct LEG*leg)         //运动学正解
{
  double a1 = (PI - leg->sensor_1.pos + leg->sensor_2.pos)/2;
  double l = L1 * cos(a1) + sqrt(L2*L2 - L1*L1 * sin(a1)*sin(a1));
  double g1 = a1 - leg->sensor_2.pos;
  double x = l * cos(g1);
  double y = (-l * sin(g1));

  leg->point_a.x = x;
  leg->point_a.y = y;
  
}

void apc_whole(struct ROBOT * robot)
{
  apc(&robot->leg_rf);
  apc(&robot->leg_lf);
  apc(&robot->leg_rb);
  apc(&robot->leg_lb);
}


void jacobian(struct LEG*leg)    //雅可比矩阵变化
{
  double g1,g2;
  double fx,fy;
  double dxa1,dxa2,dya1,dya2;
  
  
  g1 = leg->sensor_1.pos;
  g2 = leg->sensor_2.pos;
  
  fx = leg->force_g.x;
  fy = leg->force_g.y;

  dxa1 = cos(g1/2 - PI/2 + g2/2)*((L1*sin(PI/2 - g1/2 + g2/2))/2 + (L1*L1*sin(PI/2 - g1/2 + g2/2)*cos(PI/2 - g1/2 + g2/2))/(2*sqrt(- L1*L1*sin(PI/2 - g1/2 + g2/2)*sin(PI/2 - g1/2 + g2/2) + L2*L2))) - (sin(g1/2 - PI/2 + g2/2)*(L1*cos(PI/2 - g1/2 + g2/2) + sqrt(L2*L2 - L1*L1*sin(PI/2 - g1/2 + g2/2)*sin(PI/2 - g1/2 + g2/2))))/2;
  dxa2 = -(sin(g1/2 - PI/2 + g2/2)*(L1*cos(PI/2 - g1/2 + g2/2) + sqrt(L2*L2 - L1*L1*sin(PI/2 - g1/2 + g2/2)*sin(PI/2 - g1/2 + g2/2))))/2 - cos(g1/2 - PI/2 + g2/2)*((L1*sin(PI/2 - g1/2 + g2/2))/2 + (L1*L1*sin(PI/2 - g1/2 + g2/2)*cos(PI/2 - g1/2 + g2/2))/(2*sqrt(-L1*L1*sin(PI/2 - g1/2 + g2/2)*sin(PI/2 - g1/2 + g2/2) + L2*L2)));
  dya1 = (cos(g1/2 - PI/2 + g2/2)*(L1*cos(PI/2 - g1/2 + g2/2) + sqrt(L2*L2 - L1*L1*sin(PI/2 - g1/2 + g2/2)*sin(PI/2 - g1/2 + g2/2))))/2 + sin(g1/2 - PI/2 + g2/2)*((L1*sin(PI/2 - g1/2 + g2/2))/2 + (L1*L1*sin(PI/2 - g1/2 + g2/2)*cos(PI/2 - g1/2 + g2/2))/(2*sqrt(-L1*L1*sin(PI/2 - g1/2 + g2/2)*sin(PI/2 - g1/2 + g2/2) + L2*L2)));
  dya2 = (cos(g1/2 - PI/2 + g2/2)*(L1*cos(PI/2 - g1/2 + g2/2) + sqrt(L2*L2 - L1*L1*sin(PI/2 - g1/2 + g2/2)*sin(PI/2 - g1/2 + g2/2))))/2 - sin(g1/2 - PI/2 + g2/2)*((L1*sin(PI/2 - g1/2 + g2/2))/2 + (L1*L1*sin(PI/2 - g1/2 + g2/2)*cos(PI/2 - g1/2 + g2/2))/(2*sqrt(-L1*L1*sin(PI/2 - g1/2 + g2/2)*sin(PI/2 - g1/2 + g2/2) + L2*L2)));

  leg->motor_1.tor = dxa1 * fx + dya1 * fy;
  leg->motor_2.tor = dxa2 * fx + dya2 * fy;
  
}

void jacobian_whole(struct ROBOT * robot)
{
  jacobian(&robot->leg_rf);
  jacobian(&robot->leg_lf);
  jacobian(&robot->leg_rb);
  jacobian(&robot->leg_lb);
  
}


void curve_leg(struct LEG*leg)
{
  if(leg->t > T)
  {
    leg->t -= T;
  }
  
  if(leg->t >= 0*T && leg->t < 0.3*T)
  {
    leg->point_g.x = leg->curve.x_init;
    leg->point_g.y = leg->curve.y_init + ((leg->t - 0*T)/(0.3*T - 0*T))*(leg->curve.h_pos - leg->curve.y_init);
  }
  else if (leg->t >= 0.3*T && leg->t < 0.7*T)
  {
    leg->point_g.x = leg->curve.x_init + ((leg->t - 0.3*T)/(0.7*T - 0.3*T))*(leg->curve.x_lay - leg->curve.x_init);
    leg->point_g.y = leg->curve.h_pos;
  }
  else if (leg->t >= 0.7*T && leg->t <= 1.0*T)
  {
    leg->point_g.x = leg->curve.x_lay;
    leg->point_g.y = leg->curve.h_pos + ((leg->t - 0.7*T)/(1.0*T - 0.7*T))*(leg->curve.y_lay - leg->curve.h_pos);
  }
  
  //当足端提前接触地面的时候
  if(leg->t >= 0.7*T && leg->t <= 1.0*T && leg->touch == 1)
  {
    leg->point_g.y = leg->point_a.y;
  }
  
}

void curve_whole(struct ROBOT*robot)
{
  robot->leg_rf.t = robot->t;
  robot->leg_lf.t = robot->t;
  robot->leg_rb.t = robot->t;
  robot->leg_lb.t = robot->t;
  
  if(robot->leg_rf.state == 0 && robot->leg_lb.state == 0)
  {
    curve_leg(&robot->leg_rf);
    curve_leg(&robot->leg_lb);
  }
  
  if(robot->leg_lf.state == 0 && robot->leg_rb.state == 0)
  {
    curve_leg(&robot->leg_lf);
    curve_leg(&robot->leg_rb);
  }
  
}


void robot_control()             //机器人算法处理
{
  struct XYZ foot_s_f;    //支撑相足端位置
  struct XYZ foot_s_b;
  foot_s_f.x = 0;
  foot_s_f.y = 0; 
  foot_s_f.z = 0;
  foot_s_b.x = 0;
  foot_s_b.y = 0;
  foot_s_b.z = 0;
  if(rockdog_03.leg_rf.state && rockdog_03.leg_lb.state)
  {
    foot_s_f.x = rockdog_03.leg_rf.point_a.x + LENGTH;
    foot_s_f.z = (WIDE);
    foot_s_f.y = rockdog_03.leg_rf.point_a.y;
    
    foot_s_b.x = rockdog_03.leg_lb.point_a.x + (-LENGTH);
    foot_s_b.z = (-WIDE);
    foot_s_b.y = rockdog_03.leg_lb.point_a.y;
    
  }
  else if(rockdog_03.leg_lf.state && rockdog_03.leg_rb.state)
  {
    foot_s_f.x = rockdog_03.leg_lf.point_a.x + LENGTH;
    foot_s_f.z = (-WIDE);
    foot_s_f.y = rockdog_03.leg_lf.point_a.y;
    
    foot_s_b.x = rockdog_03.leg_rb.point_a.x + (-LENGTH);
    foot_s_b.z = (WIDE);
    foot_s_b.y = rockdog_03.leg_rb.point_a.y;
    
  }
  
  static double prepitch = 0;
  rockdog_03.pitch = atan((foot_s_f.y - foot_s_b.y)/(foot_s_f.x - foot_s_b.x));
  rockdog_03.dpitch = (rockdog_03.pitch - prepitch)/(0.001 * TIME_STEP);
  prepitch = rockdog_03.pitch;
  
  /*支撑相控制*************************************************/
  static double pre_h = 0.17;
  double h  = -(foot_s_f.y + foot_s_b.y)/2;
  double dh = (h - pre_h)/(0.001*(double)TIME_STEP);
  pre_h = h;
  
  //解算三维力
  double Fy = -(rockdog_03.kh*(rockdog_03.h_g - h) + rockdog_03.kdh*dh);
  double Tx = -(rockdog_03.kr*(rockdog_03.rpy_g.x - rockdog_03.rpy_a.x) + rockdog_03.kdr*rockdog_03.drpy_a.x);
  //double Tz = -(rockdog_03.kp*(rockdog_03.rpy_g.z - rockdog_03.rpy_a.z) + rockdog_03.kdp*rockdog_03.drpy_a.z);
  double Tz = -(50*(rockdog_03.pitch) + (50)*rockdog_03.dpitch);
  double Ty = -(rockdog_03.ky*(rockdog_03.drpy_g.y - rockdog_03.drpy_a.y));
  Fy += -(M*G*cos(rockdog_03.rpy_a.z));
  
  //建立前后腿的三维位置
  double xf = foot_s_f.x;
  double xh = foot_s_b.x;
  double yf = foot_s_f.y;
  double yh = foot_s_b.y;
  double zf = foot_s_f.z;
  double zh = foot_s_b.z;
  
  //将身体的三维力映射到前后腿二维力上
  double ffx = Fy*((xf*zh*zh - xh*zf*zh)/(yf*zh*zh + yh*zf*zf - yf*zf*zh - yh*zf*zh)) + Tx*((xf*zh - xh*zh)/(yf*zh*zh + yh*zf*zf - yf*zf*zh - yh*zf*zh)) + Ty*(-yh/(yf*zh - yh*zf)) + Tz*(-zh/(yf*zh - yh*zf));
  double fbx = Fy*((xh*zf*zf - xf*zh*zf)/(yf*zh*zh + yh*zf*zf - yf*zf*zh - yh*zf*zh)) + Tx*(-(xf*zf - xh*zf)/(yf*zh*zh + yh*zf*zf - yf*zf*zh - yh*zf*zh)) + Ty*(yf/(yf*zh - yh*zf)) + Tz*(zf/(yf*zh - yh*zf));
  double ffy = Fy*(-zh/(zf - zh)) + Tx*(-1/(zf - zh));
  double fby = Fy*(zf/(zf - zh)) + Tx*(1/(zf - zh));
  ffx += 1*M*G*sin(rockdog_03.rpy_a.z)/2;
  fbx += 1*M*G*sin(rockdog_03.rpy_a.z)/2;
  
  if(rockdog_03.leg_lf.state && rockdog_03.leg_rb.state)
  {
    rockdog_03.leg_lf.force_g.x = ffx;
    rockdog_03.leg_rb.force_g.x = fbx;
    rockdog_03.leg_lf.force_g.y = ffy;
    rockdog_03.leg_rb.force_g.y = fby;
    
    rockdog_03.leg_rf.force_g.x = 0;
    rockdog_03.leg_lb.force_g.x = 0;
    rockdog_03.leg_rf.force_g.y = 0;
    rockdog_03.leg_lb.force_g.y = 0;
  }
  else if(rockdog_03.leg_rf.state && rockdog_03.leg_lb.state)
  {
    rockdog_03.leg_rf.force_g.x = ffx;
    rockdog_03.leg_lb.force_g.x = fbx;
    rockdog_03.leg_rf.force_g.y = ffy;
    rockdog_03.leg_lb.force_g.y = fby;
    
    rockdog_03.leg_lf.force_g.x = 0;
    rockdog_03.leg_rb.force_g.x = 0;
    rockdog_03.leg_lf.force_g.y = 0;
    rockdog_03.leg_rb.force_g.y = 0;
  }
  
  /*摆动相控制**********************************************************/

  double x_lay = ((rockdog_03.vel_a.x*T*0.001)/2) + rockdog_03.k_a*(rockdog_03.vel_a.x-rockdog_03.vel_g.x);
  x_lay += -(1*tan(rockdog_03.rpy_g.z) * h);
  if(rockdog_03.leg_rf.state && rockdog_03.leg_lb.state)
  {
    if(rockdog_03.t <= 0.3*T)
    {
      rockdog_03.leg_lf.curve.x_lay = x_lay;
      rockdog_03.leg_rb.curve.x_lay = x_lay;
    }
  }
  else if(rockdog_03.leg_lf.state && rockdog_03.leg_rb.state)
  {
    if(rockdog_03.t <= 0.3*T)
    {
      rockdog_03.leg_rf.curve.x_lay = x_lay;
      rockdog_03.leg_lb.curve.x_lay = x_lay;
    }
  }
  
  curve_whole(&rockdog_03);
  tor_map();                        //力矩映射
  
  jacobian_whole(&rockdog_03);
  
}


void tor_map()           //力矩映射
{
  if(rockdog_03.leg_rf.state == 0)
  {
    double y_g = rockdog_03.leg_rf.point_g.y;
    double x_g = rockdog_03.leg_rf.point_g.x;
    
    static double y_pre = -0.17;
    static double x_pre = 0;
    
    double y_now = rockdog_03.leg_rf.point_a.y;
    double x_now = rockdog_03.leg_rf.point_a.x;
    
    double y_err = y_g - y_now;
    double x_err = x_g - x_now;
    
    double y_d = -(y_now - y_pre)/0.001;
    double x_d = -(x_now - x_pre)/0.001;
    
    y_pre = y_now;
    x_pre = x_now;
    
    double f_y = 4000*y_err + 100*y_d;
    double f_x = 4000*x_err + 100*x_d;
    
    rockdog_03.leg_rf.force_g.y = f_y;
    rockdog_03.leg_rf.force_g.x = f_x;
    
    if(rockdog_03.leg_rf.t <= 0.2*T)
    {
      rockdog_03.leg_rf.force_g.x = 0;
    }
  }
  
  ///////////////////////////////////////////////////////////
  if(rockdog_03.leg_lf.state == 0)
  {
    double y_g_1 = rockdog_03.leg_lf.point_g.y;
    double x_g_1 = rockdog_03.leg_lf.point_g.x;
    
    static double y_pre_1 = -0.17;
    static double x_pre_1 = 0;
    
    double y_now_1 = rockdog_03.leg_lf.point_a.y;
    double x_now_1 = rockdog_03.leg_lf.point_a.x;
    
    double y_err_1 = y_g_1 - y_now_1;
    double x_err_1 = x_g_1 - x_now_1;
    
    double y_d_1 = -(y_now_1 - y_pre_1)/0.001;
    double x_d_1 = -(x_now_1 - x_pre_1)/0.001;
    
    y_pre_1 = y_now_1;
    x_pre_1 = x_now_1;
    
    double f_y_1 = 4000*y_err_1 + 100*y_d_1;
    double f_x_1 = 4000*x_err_1 + 100*x_d_1;
    
    rockdog_03.leg_lf.force_g.y = f_y_1;
    rockdog_03.leg_lf.force_g.x = f_x_1;
    
    if(rockdog_03.leg_lf.t <= 0.2*T)
    {
      rockdog_03.leg_lf.force_g.x = 0;
    }
  }
  
  ///////////////////////////////////////////////////////////////
  if(rockdog_03.leg_rb.state == 0)
  {
    double y_g_2 = rockdog_03.leg_rb.point_g.y;
    double x_g_2 = rockdog_03.leg_rb.point_g.x;
    
    static double y_pre_2 = -0.17;
    static double x_pre_2 = 0;
    
    double y_now_2 = rockdog_03.leg_rb.point_a.y;
    double x_now_2 = rockdog_03.leg_rb.point_a.x;
    
    double y_err_2 = y_g_2 - y_now_2;
    double x_err_2 = x_g_2 - x_now_2;
    
    double y_d_2 = -(y_now_2 - y_pre_2)/0.001;
    double x_d_2 = -(x_now_2 - x_pre_2)/0.001;
    
    y_pre_2 = y_now_2;
    x_pre_2 = x_now_2;
    
    double f_y_2 = 4000*y_err_2 + 100*y_d_2;
    double f_x_2 = 4000*x_err_2 + 100*x_d_2;
    
    rockdog_03.leg_rb.force_g.y = f_y_2;
    rockdog_03.leg_rb.force_g.x = f_x_2;
    
    if(rockdog_03.leg_rb.t <= 0.2*T)
    {
      rockdog_03.leg_rb.force_g.x = 0;
    }
  }
  //////////////////////////////////////////////////////////////////
  if(rockdog_03.leg_lb.state == 0)
  {
    double y_g_3 = rockdog_03.leg_lb.point_g.y;
    double x_g_3 = rockdog_03.leg_lb.point_g.x;
    
    static double y_pre_3 = -0.17;
    static double x_pre_3 = 0;
    
    double y_now_3 = rockdog_03.leg_lb.point_a.y;
    double x_now_3 = rockdog_03.leg_lb.point_a.x;
    
    double y_err_3 = y_g_3 - y_now_3;
    double x_err_3 = x_g_3 - x_now_3;
    
    double y_d_3 = -(y_now_3 - y_pre_3)/0.001;
    double x_d_3 = -(x_now_3 - x_pre_3)/0.001;
    
    y_pre_3 = y_now_3;
    x_pre_3 = x_now_3;
    
    double f_y_3 = 4000*y_err_3 + 100*y_d_3;
    double f_x_3 = 4000*x_err_3 + 100*x_d_3;
    
    rockdog_03.leg_lb.force_g.y = f_y_3;
    rockdog_03.leg_lb.force_g.x = f_x_3;
    
    if(rockdog_03.leg_lb.t <= 0.2*T)
    {
      rockdog_03.leg_lb.force_g.x = 0;
    }
  }

  
}


#endif

