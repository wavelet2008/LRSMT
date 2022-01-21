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


lowerproxy::lowerproxy(std::string name) : Node(name){
    std::cout<<"Start to create the ros node subscriber and publisher"
                <<std::endl;
    // create a list of low level velocity command publisher 
    joint0_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>
        ("/joint0_position_controller/commands", 10);
    joint1_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>
        ("/joint1_position_controller/commands", 10);
    // joint2_velocity_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>
    //     ("/joint2_velocity_controller/commands", 10);
    // joint3_velocity_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>
    //     ("/joint3_velocity_controller/commands", 10);
    // joint4_velocity_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>
    //     ("/joint4_velocity_controller/commands", 10);
    // joint5_velocity_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>
    //     ("/joint5_velocity_controller/commands", 10);
    // joint6_velocity_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>
    //     ("/joint6_velocity_controller/commands", 10);
    // joint7_velocity_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>
    //     ("/joint7_velocity_controller/commands", 10);
    // joint8_velocity_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>
    //     ("/joint8_velocity_controller/commands", 10);
    // joint9_velocity_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>
    //     ("/joint9_velocity_controller/commands", 10);
    // joint10_velocity_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>
    //     ("/joint10_velocity_controller/commands", 10);
    // joint11_velocity_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>
    //     ("/joint11_velocity_controller/commands", 10);

    // this is the timer for automatically updates the control command
    // auto period = 500ms;
    // _timer = this->create_wall_timer
    //     (period, std::bind(&lowerproxy::PublishControlCommand, this));

    _count = 0;

    RCLCPP_INFO(this->get_logger(), "Publisher created!!");
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
    }

void lowerproxy::PublishControlCommand(){
    _count = _count + 0.01;

    auto message = std_msgs::msg::Float64MultiArray();
    message.data.push_back(_count);
    auto message2 = std_msgs::msg::Float64MultiArray();
    message2.data.push_back(2*_count);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.data.at(0));
    joint0_publisher->publish(message);
    joint1_publisher->publish(message2);
    // joint2_velocity_publisher->publish(message);
    // joint3_velocity_publisher->publish(message);
    // joint4_velocity_publisher->publish(message);
    // joint5_velocity_publisher->publish(message);
    // joint6_velocity_publisher->publish(message);
    // joint7_velocity_publisher->publish(message);
    // joint8_velocity_publisher->publish(message);
    // joint9_velocity_publisher->publish(message);
    // joint10_velocity_publisher->publish(message);
    // joint11_velocity_publisher->publish(message);
}

    
} //namespace control
} //namespace agiletaur



