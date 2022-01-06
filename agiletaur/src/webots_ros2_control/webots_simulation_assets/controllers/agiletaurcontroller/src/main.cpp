/*
 * @Author: Ryoma Liu -- ROBOLAND 
 * @Date: 2021-11-27 16:20:05 
 * @Last Modified by: Ryoma Liu
 * @Last Modified time: 2021-11-28 21:43:48
 */


#include "main.h"


/**
 * main - entrance of controller.
 * @param argc
 * @param argv
 * @return 0
 */


int main(int argc, char **argv)
{
	ControlConfig& Config = ControlConfig::GetControlConfig();
	lowerproxy &Lower_proxy = lowerproxy::GetLowerProxy();
	upperproxy &Upper_proxy = upperproxy::GetUpperProxy();
	ControllerMonitor& monitor = ControllerMonitor::GetStateMonitor();

	Config.LoadParams();
	int timeStep = (int) robot->getBasicTimeStep();

	Lower_proxy.Init(timeStep);
	Upper_proxy.Init();
	monitor.Init();
	std::cout<<"Init the robot done" <<std::endl;
	while (robot->step(timeStep) != -1){
		Lower_proxy.PublishControlCommand();
	}
	return 0;
}
