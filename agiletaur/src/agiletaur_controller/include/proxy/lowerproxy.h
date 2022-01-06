/*
 * @Author: Ryoma Liu -- ROBOLAND 
 * @Date: 2021-11-21 22:01:05 
 * @Last Modified by: Ryoma Liu
 * @Last Modified time: 2021-11-28 01:02:33
 */

#ifndef LOWERPROXY_H_
#define LOWERPROXY_H_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;
namespace agiletaur{
namespace control{

class lowerproxy:public rclcpp::Node{
  public:
    lowerproxy(std::string name = "lower_proxy");
    void PublishControlCommand();
  private:

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr 
                                                  joint0_velocity_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr 
                                                  joint1_velocity_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr 
                                                  joint2_velocity_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr 
                                                  joint3_velocity_publisher; 
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr 
                                                  joint4_velocity_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr 
                                                  joint5_velocity_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr 
                                                  joint6_velocity_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr 
                                                  joint7_velocity_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr 
                                                  joint8_velocity_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr 
                                                  joint9_velocity_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr 
                                                  joint10_velocity_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr 
                                                  joint11_velocity_publisher;                                                                                                 
    rclcpp::TimerBase::SharedPtr _timer;
    int _count;

    // const char* motor_c[8] = { "motor_4","motor_3",
    //         "motor_8","motor_7",
    //         "motor_1","motor_2",
    //         "motor_5","motor_6" };
    // const char* foot_c[4] = { "Foot_B","Foot_D","Foot_A","Foot_C" };
};

} //namespace control
} //namespace agiletaur



#endif
