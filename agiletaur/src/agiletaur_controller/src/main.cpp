/*
 * @Author: Ryoma Liu -- ROBOLAND 
 * @Date: 2021-11-27 16:20:05 
 * @Last Modified by: Ryoma Liu
 * @Last Modified time: 2021-11-28 21:43:48
 */


#include "main.h"
#include "rclcpp/rclcpp.hpp"

/**
 * main - entrance of controller.
 * @param argc
 * @param argv
 * @return 0
 */


int main(int argc, char **argv)
{

	upperproxy &Upper_proxy_ = upperproxy::GetUpperProxy();
	ControllerMonitor& monitor = ControllerMonitor::GetStateMonitor();

	rclcpp::init(argc, argv);
	std::shared_ptr<lowerproxy> Lower_proxy_ = std::make_shared<lowerproxy>();

	Upper_proxy_.Init();
	monitor.Init();
	std::cout<<"Init the robot done" <<std::endl;
	rclcpp::spin(Lower_proxy_);
  	rclcpp::shutdown();
	
	return 0;
}
