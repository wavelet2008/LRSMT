#include "wbInterface.h"
#include "locomotion_header.h"
#include "gait_math.h"

WbDeviceTag posensor[8];
WbDeviceTag motor[8];
WbDeviceTag foot[4];
WbDeviceTag IMU;
WbDeviceTag GPS;

const char* motor_c[8] = { "motor_4","motor_3",
						   "motor_8","motor_7",
						   "motor_1","motor_2",
						   "motor_5","motor_6" };

const char* foot_c[4] = { "Foot_B","Foot_D","Foot_A","Foot_C" };

//放在主函数循环之前 初始化设备
void webots_device_init(void)
{
	/* 陀螺仪设备初始化 */
	IMU = wb_robot_get_device("imu");
	wb_inertial_unit_enable(IMU, TIME_STEP);

	/* GPS设备初始化 */
	GPS = wb_robot_get_device("gps");
	wb_gps_enable(GPS, TIME_STEP);

	/* 电机及位置传感器设备初始化 */
	for (uint8_t i = 0; i < 8; i++) {
		motor[i] = wb_robot_get_device(motor_c[i]);
		posensor[i] = wb_motor_get_position_sensor(motor[i]);
		wb_position_sensor_enable(posensor[i], TIME_STEP);
		wb_motor_enable_torque_feedback(motor[i], TIME_STEP);
	}

	/* 足端触地传感器设备初始化 */
	for (uint8_t i = 0; i < 4; i++) {
		foot[i] = wb_robot_get_device(foot_c[i]);
		wb_touch_sensor_enable(foot[i], TIME_STEP);
	}

	/* 使能键盘读取 */
	wb_keyboard_enable(TIME_STEP);

	wb_joystick_enable(TIME_STEP);
}
