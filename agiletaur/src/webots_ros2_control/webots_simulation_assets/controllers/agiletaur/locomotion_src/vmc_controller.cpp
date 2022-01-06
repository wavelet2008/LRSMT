#include "include.h"
#include "gait_math.h"
#include "force_dis_8.h"
#include "locomotion_header.h"
#include "optimaize.h"
float FLT_ATT_CTRL_Y = 0.35;
float FLT_TORQUE = 25;
float FLT_ATT_SPD_ERR = 6;
float k_att_spd = 25;
float att_trig_inner[2];
float k_att_inner_small[2] = { 0,0 };
float delay_rotate = 0.025;
float fall_roll_z_gain = 0.4;
float att_stable_weight = 0;
void body_traj_planner(float dt)
{
	char i, j;
	float att_rc_off[3] = { 0 };
	float att_use[3];
	float delay_time = 0;
	float att_stable_weight_temp = (fabs(dead(robotwb.exp_att.roll - robotwb.now_att.roll, 6)) +
		fabs(dead(robotwb.exp_att.pitch + vmc_all.tar_att_off[PITr] - robotwb.now_att.pitch, 6))) / 6.0;
	DigitalLPF(1 - limitw(att_stable_weight_temp, 0.0, 0.9), &att_stable_weight, 1, dt);
	//att_stable_weight = 1;

	att_use[PITr] = vmc_all.att_vm_b[PITr];
	att_use[ROLr] = vmc_all.att_ctrl[ROLr];

	//--------------------遥控速度赋值
	vmc_all.param.tar_spd_use_rc.x = vmc_all.tar_spd.x*att_stable_weight;
	vmc_all.param.tar_spd_use_rc.y = vmc_all.tar_spd.y*att_stable_weight;
	vmc_all.param.tar_spd_use_rc.z = vmc_all.tar_spd.z*att_stable_weight;
	//printf("%f %f %f\n", vmc_all.tar_spd.x, vmc_all.param.tar_spd_use_rc.x, att_stable_weight);
	//--------------------前进速度Pitch补偿
	vmc_all.cog_off[2] = vmc_all.param.param_vmc.move_att_off[PITr];
	vmc_all.tar_att_off[PITr] = LIMIT(vmc_all.param.tar_spd_use_rc.x, -MAX_SPD, MAX_SPD) / (MAX_SPD + 0.00001)*vmc_all.cog_off[2];

	//--------------------旋转Roll补偿
	if (fabs(vmc_all.param.tar_spd_use_rc.x) > MAX_SPD*0.2&&fabs(vmc_all.tar_spd.z) > MAX_SPD_RAD*0.1)
		if (vmc_all.param.tar_spd_use_rc.x > 0)
			att_rc_off[ROLr] = -vmc_all.cog_off[3] * sign(vmc_all.param.tar_spd_use_rc.x)*LIMIT(vmc_all.tar_spd.z, -MAX_SPD_RAD * 0.35, MAX_SPD_RAD*0.35) / (MAX_SPD_RAD*0.35 + 0.00001);
		else
			att_rc_off[ROLr] = -vmc_all.cog_off[4] * sign(vmc_all.param.tar_spd_use_rc.x)*LIMIT(vmc_all.tar_spd.z, -MAX_SPD_RAD * 0.35, MAX_SPD_RAD*0.35) / (MAX_SPD_RAD*0.35 + 0.00001);
	else//原地
		att_rc_off[ROLr] = -vmc_all.cog_off[5] * LIMIT(vmc_all.tar_spd.z, -MAX_SPD_RAD * 0.35, MAX_SPD_RAD*0.35) / (MAX_SPD_RAD*0.35 + 0.00001);

	if (vmc_all.param.leg_dof == 3)
		att_rc_off[ROLr] += LIMIT(vmc_all.param.tar_spd_use_rc.y, -MAX_SPD * 0.5, MAX_SPD*0.5) / (MAX_SPD*0.5 + 0.00001)*vmc_all.cog_off[2] * 0;
	else
		att_rc_off[ROLr] += LIMIT(vmc_all.param.tar_spd_use_rc.y, -MAX_SPD * 0.5, MAX_SPD*0.5) / (MAX_SPD*0.5 + 0.00001) * 18;

	DigitalLPF(att_rc_off[ROLr], &vmc_all.tar_att_off[ROLr], 0.8, dt);

	//-----------步态延时
	if (fabs(vmc_all.param.tar_spd_use_rc.x) > 0.02) {
		delay_time = vmc_all.delay_time[0];
		DigitalLPF(delay_time, &vmc_all.delay_time[1], 0.6, dt);
	}
	else {
		if (fabs(vmc_all.tar_spd.z) < MAX_SPD_RAD*0.1) {
			delay_time = delay_rotate;
		}
		else
			delay_time = delay_rotate / 2;
		DigitalLPF(delay_time, &vmc_all.delay_time[1], 0.1, dt);
	}

	//-----------------ROLL失稳高度补偿
	float z_dis = fabs(MAX_Z) - fabs(MIN_Z);
	float z_off = -(fabs(dead(robotwb.exp_att.roll - robotwb.now_att.roll, 6)) +
		fabs(dead(robotwb.exp_att.pitch + vmc_all.tar_att_off[PITr] - robotwb.now_att.pitch, 10))) / 8.0
		*fall_roll_z_gain*z_dis;
	DigitalLPF(limitw(z_off, -fall_roll_z_gain * z_dis, 0), &vmc_all.tar_pos_off[Zr], 0.5, dt);
}



#if USE_FORCE_REAL_GROUND
float gain_tort[6] = { 1,1,1,1,1,1 };
#else
float gain_tort[6] = { 2,1,2,2,2,1 };
#endif
void body_servo_control(float dt)//机体伺服控制器
{
	char i;
	static float spdx_reg = 0;
	float att_cmd[3] = { 0 };
	float pos_cmd[3] = { 0 };
	float exp_rate_yaw = 0;
	//期望状态滤波
	if (vmc_all.gait_mode == PRONK|| vmc_all.gait_mode == CLIMB) {
		DigitalLPF(vmc_all.tar_pos.x, &robotwb.exp_pos_n.x, 5, dt);
		DigitalLPF(vmc_all.tar_pos.z, &robotwb.exp_pos_n.z, 5, dt);
	}
	else {
		DigitalLPF(vmc_all.tar_pos.x, &robotwb.exp_pos_n.x, 3, dt);
		DigitalLPF(vmc_all.tar_pos.z, &robotwb.exp_pos_n.z, 3, dt);
	}
	exp_rate_yaw = vmc_all.param.tar_spd_use_rc.z*att_stable_weight;

	//printf("vmc_all.gait_mode=%d\n", vmc_all.gait_mode);
	switch (vmc_all.gait_mode) {
	case STAND_RC:case STAND_IMU:case STAND_PUSH:case PRONK:
		if (stand_force_enable_flag[4]) {

			//Pitr-----------------------------------------
			if (vmc_all.ground_num >= 3)
				robotwb.exp_torque_i.y += limitw(robotwb.exp_att.pitch - robotwb.now_att.pitch, -15, 15) / 57.3*vmc_robot_p.att_pit.ki*dt;
			else if (vmc_all.ground_num <= 1)
				robotwb.exp_torque_i.y = 0;
			robotwb.exp_torque_i.y = limitw(robotwb.exp_torque_i.y, -robotwb.max_torque.y*0.35, robotwb.max_torque.y*0.35);

			robotwb.exp_torque.y = att_cmd[PITr] = limitw(robotwb.exp_att.pitch - robotwb.now_att.pitch, -15, 15) / 57.3*vmc_robot_p.att_pit.kp
				- robotwb.now_rate.pitch / 57.3*vmc_robot_p.att_pit.kd + robotwb.exp_torque_i.y;
			robotwb.exp_torque.y = limitw(robotwb.exp_torque.y, -robotwb.max_torque.y, robotwb.max_torque.y);

			//Rolr------------------------------------------
			if (vmc_all.ground_num >= 3)
				robotwb.exp_torque_i.x += limitw(robotwb.exp_att.roll - robotwb.now_att.roll, -25, 25) / 57.3*vmc_robot_p.att_rol.ki*dt;
			else if (vmc_all.ground_num <= 1)
				robotwb.exp_torque_i.x = 0;

			robotwb.exp_torque_i.x = limitw(robotwb.exp_torque_i.x, -robotwb.max_torque.x*0.25, robotwb.max_torque.x*0.25);

			robotwb.exp_torque.x = att_cmd[ROLr] = limitw(robotwb.exp_att.roll - robotwb.now_att.roll, -25, 25) / 57.3*vmc_robot_p.att_rol.kp
				- robotwb.now_rate.roll / 57.3*vmc_robot_p.att_rol.kd + robotwb.exp_torque_i.x;
			robotwb.exp_torque.x = limitw(robotwb.exp_torque.x, -robotwb.max_torque.x, robotwb.max_torque.x);

			robotwb.exp_torque.z = 0;

			//Xr
			if (vmc_all.ground_num >= 3)
				robotwb.exp_force_i.x += (robotwb.exp_pos_n.x - vmc_all.pos_n.x)*vmc_robot_p.pos_x.ki*dt;
			else if (vmc_all.ground_num <= 1)
				robotwb.exp_force_i.x = 0;

			robotwb.exp_force_i.x = limitw(robotwb.exp_force_i.x, -robotwb.max_force.x*0.25, robotwb.max_force.x*0.25);

			robotwb.exp_force.x = pos_cmd[Xr] = (robotwb.exp_pos_n.x - vmc_all.pos_n.x)*vmc_robot_p.pos_x.kp
				- vmc_all.spd_n.x*vmc_robot_p.pos_x.kd + robotwb.exp_force_i.x+ robotwb.cog_F_ff.x;

			//Zr
			if (vmc_all.ground_num >= 3)
				robotwb.exp_force_i.z += (fabs(robotwb.exp_pos_n.z) - vmc_all.pos_n.z)*vmc_robot_p.pos_z.ki*dt;
			else if (vmc_all.ground_num <= 1)
				robotwb.exp_force_i.z = 0;
			robotwb.exp_force_i.z = limitw(robotwb.exp_force_i.z, -robotwb.max_force.z*0.25, robotwb.max_force.z*0.25);

			robotwb.exp_force.z = pos_cmd[Zr] = (fabs(robotwb.exp_pos_n.z) - vmc_all.pos_n.z)*vmc_robot_p.pos_z.kp
				+ vmc_all.spd_n.z*vmc_robot_p.pos_z.kd + robotwb.exp_force_i.z;

			robotwb.exp_force.z += Mw * gw*vmc_robot_p.mess_scale+ robotwb.cog_F_ff.z;//重力前馈

			robotwb.exp_torque.z = robotwb.cog_T_ff.z;
			//robotwb.exp_torque.x=robotwb.exp_torque.y=robotwb.exp_torque.z=0;//测试用屏蔽姿态	
			force_dis_n();
		}
		else
		{//清除积分
			robotwb.exp_torque_i.x = robotwb.exp_torque_i.y = robotwb.exp_torque_i.z = 0;
			robotwb.exp_force_i.x = robotwb.exp_force_i.y = robotwb.exp_force_i.z = 0;

			robotwb.exp_force.x = robotwb.exp_force.y = robotwb.exp_force.z = 0;
			robotwb.exp_torque.x = robotwb.exp_torque.y = robotwb.exp_torque.z = 0;
		}
		robotwb.exp_att.yaw = robotwb.now_att.yaw;
		break;

	case BOUND:

		if (stand_force_enable_flag[4]) {
			if (gait_bound.force_control_mode == 0) {
				//Pitr-----------------------------------------
				if (vmc_all.ground_num >= 3)
					robotwb.exp_torque_i.y += limitw(robotwb.exp_att.pitch - robotwb.now_att.pitch, -15, 15) / 57.3*vmc_robot_p.att_pit.ki*dt;
				else if (vmc_all.ground_num <= 1)
					robotwb.exp_torque_i.y = 0;
				robotwb.exp_torque_i.y = limitw(robotwb.exp_torque_i.y, -robotwb.max_torque.y*0.35, robotwb.max_torque.y*0.35);

				robotwb.exp_torque.y = att_cmd[PITr] = limitw(robotwb.exp_att.pitch - robotwb.now_att.pitch, -15, 15) / 57.3*vmc_robot_p.att_pit.kp
					- robotwb.now_rate.pitch / 57.3*vmc_robot_p.att_pit.kd + robotwb.exp_torque_i.y;
				robotwb.exp_torque.y = limitw(robotwb.exp_torque.y, -robotwb.max_torque.y, robotwb.max_torque.y);

				//Rolr------------------------------------------
				if (vmc_all.ground_num >= 3)
					robotwb.exp_torque_i.x += limitw(robotwb.exp_att.roll - robotwb.now_att.roll, -25, 25) / 57.3*vmc_robot_p.att_rol.ki*dt;
				else if (vmc_all.ground_num <= 1)
					robotwb.exp_torque_i.x = 0;

				robotwb.exp_torque_i.x = limitw(robotwb.exp_torque_i.x, -robotwb.max_torque.x*0.25, robotwb.max_torque.x*0.25);

				robotwb.exp_torque.x = att_cmd[ROLr] = limitw(robotwb.exp_att.roll - robotwb.now_att.roll, -25, 25) / 57.3*vmc_robot_p.att_rol.kp
					- robotwb.now_rate.roll / 57.3*vmc_robot_p.att_rol.kd + robotwb.exp_torque_i.x;
				robotwb.exp_torque.x = limitw(robotwb.exp_torque.x, -robotwb.max_torque.x, robotwb.max_torque.x);

				robotwb.exp_torque.z = 0;

				//Xr
				if (vmc_all.ground_num >= 3)
					robotwb.exp_force_i.x += (robotwb.exp_pos_n.x - vmc_all.pos_n.x)*vmc_robot_p.pos_x.ki*dt;
				else if (vmc_all.ground_num <= 1)
					robotwb.exp_force_i.x = 0;

				robotwb.exp_force_i.x = limitw(robotwb.exp_force_i.x, -robotwb.max_force.x*0.25, robotwb.max_force.x*0.25);

				robotwb.exp_force.x = pos_cmd[Xr] = (robotwb.exp_pos_n.x - vmc_all.pos_n.x)*vmc_robot_p.pos_x.kp
					- vmc_all.spd_n.x*vmc_robot_p.pos_x.kd + robotwb.exp_force_i.x + robotwb.cog_F_ff.x;

				//Zr
				if (vmc_all.ground_num >= 3)
					robotwb.exp_force_i.z += (fabs(robotwb.exp_pos_n.z) - vmc_all.pos_n.z)*vmc_robot_p.pos_z.ki*dt;
				else if (vmc_all.ground_num <= 1)
					robotwb.exp_force_i.z = 0;
				robotwb.exp_force_i.z = limitw(robotwb.exp_force_i.z, -robotwb.max_force.z*0.25, robotwb.max_force.z*0.25);

				robotwb.exp_force.z = pos_cmd[Zr] = (fabs(robotwb.exp_pos_n.z) - vmc_all.pos_n.z)*vmc_robot_p.pos_z.kp
					+ vmc_all.spd_n.z*vmc_robot_p.pos_z.kd + robotwb.exp_force_i.z;

				robotwb.exp_force.z += Mw * gw*vmc_robot_p.mess_scale + robotwb.cog_F_ff.z;//重力前馈

				robotwb.exp_torque.z = robotwb.cog_T_ff.z;
				//robotwb.exp_torque.x=robotwb.exp_torque.y=robotwb.exp_torque.z=0;//测试用屏蔽姿态	
				force_dis_n();
			}
			else {//前后腿独立控制模式
				if (gait_bound.force_control_mode == 2)
				{
				}
				else
				{
					//Xr
					for (i = 0; i < 2; i++) {
						robotwb.exp_force_i_b[i].x += (robotwb.exp_pos_n_b[i].x - vmc_all.pos_n_b[i].x)*vmc_robot_p.pos_x.ki*dt;

						robotwb.exp_force_i_b[i].x = limitw(robotwb.exp_force_i_b[i].x, -robotwb.max_force.x*0.25, robotwb.max_force.x*0.25);

						robotwb.exp_force_b[i].x = (robotwb.exp_pos_n_b[i].x - vmc_all.pos_n_b[i].x)*vmc_robot_p.pos_x.kp
							- vmc_all.spd_n_b[i].x*vmc_robot_p.pos_x.kd + robotwb.exp_force_i_b[i].x;
						robotwb.exp_force_b[i].x *= 0.5;
					}
					for (i = 0; i < 2; i++) {
						//Zr
						robotwb.exp_force_i_b[i].z += (fabs(robotwb.exp_pos_n_b[i].z) - vmc_all.pos_n_b[i].z)*vmc_robot_p.pos_z.ki*dt;

						robotwb.exp_force_i_b[i].z = limitw(robotwb.exp_force_i_b[i].z, -robotwb.max_force.z*0.25, robotwb.max_force.z*0.25);

						robotwb.exp_force_b[i].z = (fabs(robotwb.exp_pos_n_b[i].z) - vmc_all.pos_n_b[i].z)*vmc_robot_p.pos_z.kp
							+ vmc_all.spd_n_b[i].z*vmc_robot_p.pos_z.kd + robotwb.exp_force_i_b[i].z;
						robotwb.exp_force_b[i].z *= 0.5;
						if (vmc_all.ground_num >= 3)
							robotwb.exp_force_b[i].z += Mw * gw*vmc_robot_p.mess_scale / 2;//重力前馈
						else
							robotwb.exp_force_b[i].z += Mw * gw*vmc_robot_p.mess_scale;//重力前馈
					}
				}
				//Rolr------------------------------------------
				if (vmc_all.ground_num >= 3)
					robotwb.exp_torque_i.x += limitw(robotwb.exp_att.roll - robotwb.now_att.roll, -25, 25) / 57.3*vmc_robot_p.att_rol.ki*dt;
				else if (vmc_all.ground_num <= 1)
					robotwb.exp_torque_i.x = 0;

				robotwb.exp_torque_i.x = limitw(robotwb.exp_torque_i.x, -robotwb.max_torque.x*0.25, robotwb.max_torque.x*0.25);

				robotwb.exp_torque.x = att_cmd[ROLr] = limitw(robotwb.exp_att.roll - robotwb.now_att.roll, -25, 25) / 57.3*vmc_robot_p.att_rol.kp
					- robotwb.now_rate.roll / 57.3*vmc_robot_p.att_rol.kd + robotwb.exp_torque_i.x;
				robotwb.exp_torque.x = limitw(robotwb.exp_torque.x, -robotwb.max_torque.x, robotwb.max_torque.x);

				//printf("exp_z=%f %f\n", fabs(robotwb.exp_pos_n_b[F].z), vmc_all.pos_n_b[F].z);
				force_dis_n_side();
			}
		}
		else
		{//清除积分
			robotwb.exp_torque_i.x = robotwb.exp_torque_i.y = robotwb.exp_torque_i.z = 0;
			robotwb.exp_force_i.x = robotwb.exp_force_i.y = robotwb.exp_force_i.z = 0;

			robotwb.exp_force.x = robotwb.exp_force.y = robotwb.exp_force.z = 0;
			robotwb.exp_torque.x = robotwb.exp_torque.y = robotwb.exp_torque.z = 0;
		}
		robotwb.exp_att.yaw = robotwb.now_att.yaw;
		break;

	case CLIMB:
		if (stand_force_enable_flag[4]) {
			if (gait_climb.force_control_mode == 0) {
				//Pitr-----------------------------------------
				if (vmc_all.ground_num >= 3)
					robotwb.exp_torque_i.y += limitw(robotwb.exp_att.pitch - robotwb.now_att.pitch, -15, 15) / 57.3*vmc_robot_p.att_pit.ki*dt;
				else if (vmc_all.ground_num <= 1)
					robotwb.exp_torque_i.y = 0;
				robotwb.exp_torque_i.y = limitw(robotwb.exp_torque_i.y, -robotwb.max_torque.y*0.35, robotwb.max_torque.y*0.35);

				robotwb.exp_torque.y = att_cmd[PITr] = limitw(robotwb.exp_att.pitch - robotwb.now_att.pitch, -15, 15) / 57.3*vmc_robot_p.att_pit.kp
					- robotwb.now_rate.pitch / 57.3*vmc_robot_p.att_pit.kd + robotwb.exp_torque_i.y;
				robotwb.exp_torque.y = limitw(robotwb.exp_torque.y, -robotwb.max_torque.y, robotwb.max_torque.y);

				//Rolr------------------------------------------
				if (vmc_all.ground_num >= 3)
					robotwb.exp_torque_i.x += limitw(robotwb.exp_att.roll - robotwb.now_att.roll, -25, 25) / 57.3*vmc_robot_p.att_rol.ki*dt;
				else if (vmc_all.ground_num <= 1)
					robotwb.exp_torque_i.x = 0;

				robotwb.exp_torque_i.x = limitw(robotwb.exp_torque_i.x, -robotwb.max_torque.x*0.25, robotwb.max_torque.x*0.25);

				robotwb.exp_torque.x = att_cmd[ROLr] = limitw(robotwb.exp_att.roll - robotwb.now_att.roll, -25, 25) / 57.3*vmc_robot_p.att_rol.kp
					- robotwb.now_rate.roll / 57.3*vmc_robot_p.att_rol.kd + robotwb.exp_torque_i.x;
				robotwb.exp_torque.x = limitw(robotwb.exp_torque.x, -robotwb.max_torque.x, robotwb.max_torque.x);

				robotwb.exp_torque.z = 0;

				//Xr
				if (vmc_all.ground_num >= 3)
					robotwb.exp_force_i.x += (robotwb.exp_pos_n.x - vmc_all.pos_n.x)*vmc_robot_p.pos_x.ki*dt;
				else if (vmc_all.ground_num <= 1)
					robotwb.exp_force_i.x = 0;

				robotwb.exp_force_i.x = limitw(robotwb.exp_force_i.x, -robotwb.max_force.x*0.25, robotwb.max_force.x*0.25);

				robotwb.exp_force.x = pos_cmd[Xr] = (robotwb.exp_pos_n.x - vmc_all.pos_n.x)*vmc_robot_p.pos_x.kp
					- vmc_all.spd_n.x*vmc_robot_p.pos_x.kd + robotwb.exp_force_i.x + robotwb.cog_F_ff.x;

				//Zr
				if (vmc_all.ground_num >= 3)
					robotwb.exp_force_i.z += (fabs(robotwb.exp_pos_n.z) - vmc_all.pos_n.z)*vmc_robot_p.pos_z.ki*dt;
				else if (vmc_all.ground_num <= 1)
					robotwb.exp_force_i.z = 0;
				robotwb.exp_force_i.z = limitw(robotwb.exp_force_i.z, -robotwb.max_force.z*0.25, robotwb.max_force.z*0.25);

				robotwb.exp_force.z = pos_cmd[Zr] = (fabs(robotwb.exp_pos_n.z) - vmc_all.pos_n.z)*vmc_robot_p.pos_z.kp
					+ vmc_all.spd_n.z*vmc_robot_p.pos_z.kd + robotwb.exp_force_i.z;

				robotwb.exp_force.z += Mw * gw*vmc_robot_p.mess_scale + robotwb.cog_F_ff.z;//重力前馈

				robotwb.exp_torque.z = robotwb.cog_T_ff.z;
				//robotwb.exp_torque.x=robotwb.exp_torque.y=robotwb.exp_torque.z=0;//测试用屏蔽姿态	
				force_dis_n();
			}
			else {//前后腿独立控制模式
					
				//Xr
				for (i = 0; i < 2; i++) {
					robotwb.exp_force_i_b[i].x += (robotwb.exp_pos_n_b[i].x - vmc_all.pos_n_b[i].x)*vmc_robot_p.pos_x.ki*dt;

					robotwb.exp_force_i_b[i].x = limitw(robotwb.exp_force_i_b[i].x, -robotwb.max_force.x*0.25, robotwb.max_force.x*0.25);

					robotwb.exp_force_b[i].x = (robotwb.exp_pos_n_b[i].x - vmc_all.pos_n_b[i].x)*vmc_robot_p.pos_x.kp
						- vmc_all.spd_n_b[i].x*vmc_robot_p.pos_x.kd + robotwb.exp_force_i_b[i].x;
					robotwb.exp_force_b[i].x *= 0.5;
				}
				for (i = 0; i < 2; i++) {
					//Zr
					robotwb.exp_force_i_b[i].z += (fabs(robotwb.exp_pos_n_b[i].z) - vmc_all.pos_n_b[i].z)*vmc_robot_p.pos_z.ki*dt;

					robotwb.exp_force_i_b[i].z = limitw(robotwb.exp_force_i_b[i].z, -robotwb.max_force.z*0.25, robotwb.max_force.z*0.25);

					robotwb.exp_force_b[i].z = (fabs(robotwb.exp_pos_n_b[i].z) - vmc_all.pos_n_b[i].z)*vmc_robot_p.pos_z.kp
						+ vmc_all.spd_n_b[i].z*vmc_robot_p.pos_z.kd + robotwb.exp_force_i_b[i].z;
					robotwb.exp_force_b[i].z *= 0.5;
					if(vmc_all.ground_num>=3)
						robotwb.exp_force_b[i].z += Mw * gw*vmc_robot_p.mess_scale/2;//重力前馈
					else
						robotwb.exp_force_b[i].z += Mw * gw*vmc_robot_p.mess_scale;//重力前馈
				}

				//Rolr------------------------------------------
				if (vmc_all.ground_num >= 3)
					robotwb.exp_torque_i.x += limitw(robotwb.exp_att.roll - robotwb.now_att.roll, -25, 25) / 57.3*vmc_robot_p.att_rol.ki*dt;
				else if (vmc_all.ground_num <= 1)
					robotwb.exp_torque_i.x = 0;

				robotwb.exp_torque_i.x = limitw(robotwb.exp_torque_i.x, -robotwb.max_torque.x*0.25, robotwb.max_torque.x*0.25);

				robotwb.exp_torque.x = att_cmd[ROLr] = limitw(robotwb.exp_att.roll - robotwb.now_att.roll, -25, 25) / 57.3*vmc_robot_p.att_rol.kp
					- robotwb.now_rate.roll / 57.3*vmc_robot_p.att_rol.kd + robotwb.exp_torque_i.x;
				robotwb.exp_torque.x = limitw(robotwb.exp_torque.x, -robotwb.max_torque.x, robotwb.max_torque.x);

				//printf("exp_z=%f %f\n", fabs(robotwb.exp_pos_n_b[F].z), vmc_all.pos_n_b[F].z);
				force_dis_n_side();
			}
		}
		else
		{//清除积分
			robotwb.exp_torque_i.x = robotwb.exp_torque_i.y = robotwb.exp_torque_i.z = 0;
			robotwb.exp_force_i.x = robotwb.exp_force_i.y = robotwb.exp_force_i.z = 0;

			robotwb.exp_force.x = robotwb.exp_force.y = robotwb.exp_force.z = 0;
			robotwb.exp_torque.x = robotwb.exp_torque.y = robotwb.exp_torque.z = 0;
		}
		robotwb.exp_att.yaw = robotwb.now_att.yaw;
		break;
	case TROT:case F_TROT://----------------------------------------------------	
		if (stand_force_enable_flag[4]) {
			//Pitr-----------------------------------------
			if (vmc_all.ground_num >= 2)
				robotwb.exp_torque_i.y += limitw(robotwb.exp_att.pitch + vmc_all.tar_att_off[PITr] - robotwb.now_att.pitch, -25, 25) / 57.3*vmc_robot_p.att_pit_trot.ki*dt;

			robotwb.exp_torque_i.y = limitw(robotwb.exp_torque_i.y, -robotwb.max_torque.y*0.2, robotwb.max_torque.y*0.2);

			robotwb.exp_torque.y = att_cmd[PITr] = limitw(robotwb.exp_att.pitch + vmc_all.tar_att_off[PITr] - robotwb.now_att.pitch, -25, 25) / 57.3*vmc_robot_p.att_pit_trot.kp
				- robotwb.now_rate.pitch / 57.3*vmc_robot_p.att_pit_trot.kd + robotwb.exp_torque_i.y;
			robotwb.exp_torque.y = limitw(robotwb.exp_torque.y, -robotwb.max_torque.y, robotwb.max_torque.y);

			//Rolr------------------------------------------
			if (vmc_all.ground_num >= 2)
				robotwb.exp_torque_i.x += limitw(robotwb.exp_att.roll - robotwb.now_att.roll, -25, 25) / 57.3*vmc_robot_p.att_rol_trot.ki*dt;


			robotwb.exp_torque_i.x = limitw(robotwb.exp_torque_i.x, -robotwb.max_torque.x*0.2, robotwb.max_torque.x*0.2);

			robotwb.exp_torque.x = att_cmd[ROLr] = limitw(robotwb.exp_att.roll - robotwb.now_att.roll, -25, 25) / 57.3*vmc_robot_p.att_rol_trot.kp
				- robotwb.now_rate.roll / 57.3*vmc_robot_p.att_rol_trot.kd + robotwb.exp_torque_i.x;
			robotwb.exp_torque.x = limitw(robotwb.exp_torque.x, -robotwb.max_torque.x, robotwb.max_torque.x);


			//YAWr------------------------------------------
			if (fabs(exp_rate_yaw) > 1)
			{
				robotwb.exp_rate.yaw = exp_rate_yaw;
				robotwb.exp_att.yaw = robotwb.now_att.yaw;
			}
			else {
				robotwb.exp_rate.yaw = -dead(limitw(To_180_degrees(robotwb.exp_att.yaw - robotwb.now_att.yaw), -25, 25), 0.5)*vmc_robot_p.att_yaw_trot.kp_o;
			}

			if (vmc_all.ground_num >= 2)
				robotwb.exp_torque_i.z += limitw(robotwb.exp_rate.yaw - robotwb.now_rate.yaw, -15, 15) / 57.3*vmc_robot_p.att_yaw_trot.ki*dt;

			robotwb.exp_torque_i.z = limitw(robotwb.exp_torque_i.z, -robotwb.max_torque.z*0.2, robotwb.max_torque.z*0.2);

			robotwb.exp_torque.z = att_cmd[YAWr] = limitw(robotwb.exp_rate.yaw - robotwb.now_rate.yaw, -15, 15) / 57.3*vmc_robot_p.att_yaw_trot.kp
				- (robotwb.now_rate.yaw - robotwb.now_rate_reg.yaw) / dt / 57.3*vmc_robot_p.att_yaw_trot.kd
				+ robotwb.exp_torque_i.z
				+ robotwb.exp_rate.yaw / 57.3*vmc_robot_p.att_yaw_trot.vff;
			robotwb.exp_torque.z = limitw(robotwb.exp_torque.z, -robotwb.max_torque.z, robotwb.max_torque.z);
			//robotwb.exp_torque.z=0;	 

			//Xr	
			robotwb.exp_spd_n.x = vmc_all.param.tar_spd_use_rc.x;
			if (vmc_all.ground_num >= 2)
				robotwb.exp_force_i.x += (robotwb.exp_spd_n.x - vmc_all.spd_n.x)*vmc_robot_p.spd_x.ki*dt;
			else
				robotwb.exp_force_i.x = 0;

			if (!vmc_robot_p.spd_x.ki)robotwb.exp_force_i.x = 0;
			robotwb.exp_force_i.x = limitw(robotwb.exp_force_i.x, -robotwb.max_force.x*0.2, robotwb.max_force.x*0.2);

			robotwb.exp_force.x = pos_cmd[Xr] = (robotwb.exp_spd_n.x - vmc_all.spd_n.x)*vmc_robot_p.spd_x.kp + robotwb.exp_force_i.x
				- (vmc_all.spd_n.x - spdx_reg) / dt * vmc_robot_p.spd_x.kd;

			//Zr
			if (vmc_all.ground_num >= 2)
				robotwb.exp_force_i.z += (fabs(robotwb.exp_pos_n.z) + vmc_all.tar_pos_off[Zr] - vmc_all.pos_n.z)*vmc_robot_p.pos_z.ki*dt;

			robotwb.exp_force_i.z = limitw(robotwb.exp_force_i.z, -robotwb.max_force.z*0.3, robotwb.max_force.z*0.3);

			robotwb.exp_force.z = pos_cmd[Zr] = (fabs(robotwb.exp_pos_n.z) + vmc_all.tar_pos_off[Zr] - vmc_all.pos_n.z)*vmc_robot_p.pos_z.kp
				+ vmc_all.spd_n.z*vmc_robot_p.pos_z.kd + robotwb.exp_force_i.z;

			//robotwb.exp_torque.x=robotwb.exp_torque.y=robotwb.exp_torque.z=0;//测试用屏蔽姿态	

			robotwb.exp_force.x = (pos_cmd[Xr] + 0 * Mw*gw*limitw(sind(vmc_all.ground_att_est[PITr]), -0.35, 0.35)*vmc_robot_p.mess_scale);//地形重力补偿
			robotwb.exp_force.y = (pos_cmd[Yr]);
			robotwb.exp_force.z = (pos_cmd[Zr] + Mw * gw*vmc_robot_p.mess_scale);
			robotwb.exp_torque.y = (att_cmd[PITr]);
			robotwb.exp_torque.x = (att_cmd[ROLr]);
			robotwb.exp_torque.z = (att_cmd[YAWr]);

			force_dis_n();
		}
		else
		{//清除积分
			robotwb.exp_torque_i.x = robotwb.exp_torque_i.y = robotwb.exp_torque_i.z = 0;
			robotwb.exp_force_i.x = robotwb.exp_force_i.y = robotwb.exp_force_i.z = 0;

			robotwb.exp_force.x = robotwb.exp_force.y = robotwb.exp_force.z = 0;
			robotwb.exp_torque.x = robotwb.exp_torque.y = robotwb.exp_torque.z = 0;
		}
		break;
	default://清除积分
		robotwb.exp_force.x = robotwb.exp_force.y = robotwb.exp_force.z = 0;
		robotwb.exp_torque.x = robotwb.exp_torque.y = robotwb.exp_torque.z = 0;
		robotwb.exp_torque_i.x = robotwb.exp_torque_i.y = robotwb.exp_torque_i.z = 0;
		robotwb.exp_force_i.x = robotwb.exp_force_i.y = robotwb.exp_force_i.z = 0;
		for (i = 0; i < 4; i++)
			robotwb.Leg[i].tar_force_dis_n_reg.x = robotwb.Leg[i].tar_force_dis_n_reg.y = robotwb.Leg[i].tar_force_dis_n_reg.z = 0;
		break;
	}

	robotwb.now_rate_reg = robotwb.now_rate;
	spdx_reg = vmc_all.spd_n.x;
}


float GRF_OUT_FLT = 1.0;//足底力期望滤波
char MODE_ST[7] = { 0,0,0,0,0,0,0 };//Pitch Roll Nag Scale
char MODE_TORT[7] = { 0,0,0,0,0,0,0 };//Pitch Roll Nag Scale
#if USE_FORCE_REAL_GROUND
char CAL_MODE[4] = { 0,2,0,0 };//0 0均能
#else
char CAL_MODE[4] = { 0,0,0,0 };//0 0均能
#endif
//2_slove:: 0融合 1SLIP（姿态最优） 2全维（无法转向） 3最小二乘
//4_slove:: 0融合 1NC               2全维（无法转向） 3最小二乘no
double Div_Roll_ST = 0.5;//权重下横滚控制不住
double Div_Roll_TORT = 0.5;//权重下横滚控制不住
float flt_t_dt = 20;//能解决着地腿抖
void force_dis_n(void)//-------------------力分配
{
	// %     逆+（从箭头往里看）
	// %      z   y 逆+
	// %      |  /
	// %      | /
	// %      |/
	// %      o—————x 逆+
	// %1FL0   2FR1
	// %
	// %4BL3   3BR2
	static float yaw_control_mask = 0;
	int id_swap[4] = { 2,0,1,3 }, i = 0;
	double FALL[3] = { 0 }, TALL[3] = { 0 }, Fp[2] = { 0 }, PC[3] = { 0 }, P1[3] = { 0 }, P2[3] = { 0 }, P3[3] = { 0 }, P4[3] = { 0 };
	char G[4] = { 0 };
	char G_NUM = 0;
	double ATT_NOW[3] = { 0,0,0 };
	//	#endif
	double F1c[3] = { 0 }, F2c[3] = { 0 }, F3c[3] = { 0 }, F4c[3] = { 0 };
#if !USE_QP
	FALL[Xrw] = robotwb.exp_force.y;
	FALL[Yrw] = robotwb.exp_force.x;
	FALL[Zrw] = robotwb.exp_force.z;
	TALL[Xrw] = robotwb.exp_torque.y;
	TALL[Yrw] = robotwb.exp_torque.x;
	TALL[Zrw] = robotwb.exp_torque.z;

	// %1FL0   2FR1
	// %
	// %4BL3   3BR2
#if USE_FORCE_REAL_GROUND//力分配结果最终输出 最终按实际着地情况
#if  0
	G[0] = stand_force_enable_flag[id_swap[0]];
	G[1] = stand_force_enable_flag[id_swap[1]];
	G[2] = stand_force_enable_flag[id_swap[2]];
	G[3] = stand_force_enable_flag[id_swap[3]];
#else
	G[0] = robotwb.Leg[id_swap[0]].is_ground;
	G[1] = robotwb.Leg[id_swap[1]].is_ground;
	G[2] = robotwb.Leg[id_swap[2]].is_ground;
	G[3] = robotwb.Leg[id_swap[3]].is_ground;
	// printf("%d %d %d %d\n",G[0],G[1],G[2],G[3]);
	G_NUM = G[0] + G[1] + G[2] + G[3];
#if 0
	if (G[0] && G[1] && G[2] && !G[3])//核心区别  
		G[1] = 0;
	if (!G[0] && G[1] && G[2] && G[3])
		G[2] = 0;
	if (G[0] && !G[1] && G[2] && G[3])
		G[3] = 0;
	if (G[0] && G[1] && !G[2] && G[3])
		G[0] = 0;
#endif
	if (vmc_all.gait_mode == TROT && 1) {
		if (G_NUM == 4)//delay
		{
			FALL[Xrw] = gain_tort[Xrw] * robotwb.exp_force.y;
			FALL[Yrw] = gain_tort[Yrw] * robotwb.exp_force.x;
			FALL[Zrw] = gain_tort[Zrw] * robotwb.exp_force.z;
			TALL[Xrw] = gain_tort[Xrw] * robotwb.exp_torque.y;
			TALL[Yrw] = gain_tort[Yrw] * robotwb.exp_torque.x;
			TALL[Zrw] = gain_tort[Zrw] * robotwb.exp_torque.z;
			CAL_MODE[3] = 0;
		}
		else {
			FALL[Xrw] = robotwb.exp_force.y;
			FALL[Yrw] = robotwb.exp_force.x;
			FALL[Zrw] = robotwb.exp_force.z;
			TALL[Xrw] = robotwb.exp_torque.y;
			TALL[Yrw] = robotwb.exp_torque.x;
			TALL[Zrw] = robotwb.exp_torque.z;
		}
	}
#endif
#else
	G[0] = robotwb.Leg[id_swap[0]].is_ground;
	G[1] = robotwb.Leg[id_swap[1]].is_ground;
	G[2] = robotwb.Leg[id_swap[2]].is_ground;
	G[3] = robotwb.Leg[id_swap[3]].is_ground;
	G_NUM = G[0] + G[1] + G[2] + G[3];


	if (vmc_all.gait_mode == TROT && 1) {

		if (G_NUM == 4)//delay
		{
			FALL[Xrw] = robotwb.exp_force.y;
			FALL[Yrw] = robotwb.exp_force.x;
			FALL[Zrw] = robotwb.exp_force.z;
			TALL[Xrw] = robotwb.exp_torque.y;
			TALL[Yrw] = robotwb.exp_torque.x;
			TALL[Zrw] = robotwb.exp_torque.z;
		}
		else {
			FALL[Xrw] = gain_tort[Xrw] * robotwb.exp_force.y;
			FALL[Yrw] = gain_tort[Yrw] * robotwb.exp_force.x;
			FALL[Zrw] = gain_tort[Zrw] * robotwb.exp_force.z;
			TALL[Xrw] = gain_tort[Xrw] * robotwb.exp_torque.y;
			TALL[Yrw] = gain_tort[Yrw] * robotwb.exp_torque.x;
			TALL[Zrw] = gain_tort[Zrw] * robotwb.exp_torque.z;
		}
		G[0] = G[1] = G[2] = G[3] = 1;
	}

#endif

#if 1
	if (vmc_all.param.leg_dof == 2) {
		Fp[0] = FALL[Yrw] - TALL[Zrw] / Www * !USE_FORCE_REAL_GROUND;//航向控制
		Fp[1] = FALL[Yrw] + TALL[Zrw] / Www * !USE_FORCE_REAL_GROUND;
		//printf("%f\n", TALL[Zrw]);
		TALL[Zrw] *= USE_FORCE_REAL_GROUND * 0.5;
	}
#else
	Fp[0] = FALL[Yrw];
	Fp[1] = FALL[Yrw];
#endif

	PC[Xrw] = 0;
	PC[Yrw] = 0;
	PC[Zrw] = 0;

	P1[Xrw] = robotwb.Leg[id_swap[0]].epos_n.y;
	P1[Yrw] = robotwb.Leg[id_swap[0]].epos_n.x;
	P1[Zrw] = robotwb.Leg[id_swap[0]].epos_n.z;

	P2[Xrw] = robotwb.Leg[id_swap[1]].epos_n.y;
	P2[Yrw] = robotwb.Leg[id_swap[1]].epos_n.x;
	P2[Zrw] = robotwb.Leg[id_swap[1]].epos_n.z;

	P3[Xrw] = robotwb.Leg[id_swap[2]].epos_n.y;
	P3[Yrw] = robotwb.Leg[id_swap[2]].epos_n.x;
	P3[Zrw] = robotwb.Leg[id_swap[2]].epos_n.z;

	P4[Xrw] = robotwb.Leg[id_swap[3]].epos_n.y;
	P4[Yrw] = robotwb.Leg[id_swap[3]].epos_n.x;
	P4[Zrw] = robotwb.Leg[id_swap[3]].epos_n.z;

	if (vmc_all.gait_mode == TROT && 1) {
		force_dis_8new(FALL, TALL, Fp, PC, P1, P2, P3,
			P4, G, ATT_NOW, Div_Roll_TORT, MODE_TORT, CAL_MODE, F1c, F2c, F3c, F4c);
	}
	else {//站立
		TALL[Zrw]= robotwb.exp_torque.z;
		force_dis_8new(FALL, TALL, Fp, PC, P1, P2, P3,
			P4, G, ATT_NOW, Div_Roll_ST, MODE_ST, CAL_MODE, F1c, F2c, F3c, F4c);
	}
#endif

#if USE_QP&&1
	G[0] = robotwb.Leg[id_swap[0]].is_ground*stand_force_enable_flag[4];
	G[1] = robotwb.Leg[id_swap[1]].is_ground*stand_force_enable_flag[4];
	G[2] = robotwb.Leg[id_swap[2]].is_ground*stand_force_enable_flag[4];
	G[3] = robotwb.Leg[id_swap[3]].is_ground*stand_force_enable_flag[4];
	QP_Dis_Force(0.1, robotwb.max_force.z);
	F1c[Yrw] = robotwb.Leg[id_swap[0]].tar_force_dis_n_qp.x;
	F1c[Xrw] = robotwb.Leg[id_swap[0]].tar_force_dis_n_qp.y;
	F1c[Zrw] = robotwb.Leg[id_swap[0]].tar_force_dis_n_qp.z;

	F2c[Yrw] = robotwb.Leg[id_swap[1]].tar_force_dis_n_qp.x;
	F2c[Xrw] = robotwb.Leg[id_swap[1]].tar_force_dis_n_qp.y;
	F2c[Zrw] = robotwb.Leg[id_swap[1]].tar_force_dis_n_qp.z;

	F3c[Yrw] = robotwb.Leg[id_swap[2]].tar_force_dis_n_qp.x;
	F3c[Xrw] = robotwb.Leg[id_swap[2]].tar_force_dis_n_qp.y;
	F3c[Zrw] = robotwb.Leg[id_swap[2]].tar_force_dis_n_qp.z;

	F4c[Yrw] = robotwb.Leg[id_swap[3]].tar_force_dis_n_qp.x;
	F4c[Xrw] = robotwb.Leg[id_swap[3]].tar_force_dis_n_qp.y;
	F4c[Zrw] = robotwb.Leg[id_swap[3]].tar_force_dis_n_qp.z;
#endif

	float temp_min_z = pos_force_p.load_fz;
	if (vmc_all.gait_mode == TROT) {
#if 1//力分配结果最终输出 最终按实际着地情况
		G[0] = robotwb.Leg[id_swap[0]].is_ground*stand_force_enable_flag[4];
		G[1] = robotwb.Leg[id_swap[1]].is_ground*stand_force_enable_flag[4];
		G[2] = robotwb.Leg[id_swap[2]].is_ground*stand_force_enable_flag[4];
		G[3] = robotwb.Leg[id_swap[3]].is_ground*stand_force_enable_flag[4];
#else
		G[0] = stand_force_enable_flag[id_swap[0]] * stand_force_enable_flag[4];//*vmc[id_swap[0]].is_touch;;
		G[1] = stand_force_enable_flag[id_swap[1]] * stand_force_enable_flag[4];//*vmc[id_swap[1]].is_touch;;
		G[2] = stand_force_enable_flag[id_swap[2]] * stand_force_enable_flag[4];//*vmc[id_swap[2]].is_touch;;
		G[3] = stand_force_enable_flag[id_swap[3]] * stand_force_enable_flag[4];//*vmc[id_swap[3]].is_touch;;		
#endif

		robotwb.Leg[id_swap[0]].tar_force_dis_n.x = limitw(F1c[Yrw] * G[0], -robotwb.max_force.x, robotwb.max_force.x);
		robotwb.Leg[id_swap[0]].tar_force_dis_n.y = F1c[Xrw] * G[0];
		robotwb.Leg[id_swap[0]].tar_force_dis_n.z = limitw(F1c[Zrw] * G[0], temp_min_z, robotwb.max_force.z);
		if (vmc[id_swap[0]].param.trig_state == 4 && EN_TORT_LOAD_FORCE4) {//等待时load力
			robotwb.Leg[id_swap[0]].tar_force_dis_n.x = 0;
			robotwb.Leg[id_swap[0]].tar_force_dis_n.z = temp_min_z;
		}
		else if (vmc[id_swap[0]].param.trig_state == 5 && EN_TORT_LOAD_FORCE5) {//等待时load力
			robotwb.Leg[id_swap[0]].tar_force_dis_n.x = 0;
			robotwb.Leg[id_swap[0]].tar_force_dis_n.z = temp_min_z;
		}

		robotwb.Leg[id_swap[1]].tar_force_dis_n.x = limitw(F2c[Yrw] * G[1], -robotwb.max_force.x, robotwb.max_force.x);
		robotwb.Leg[id_swap[1]].tar_force_dis_n.y = F2c[Xrw] * G[1];
		robotwb.Leg[id_swap[1]].tar_force_dis_n.z = limitw(F2c[Zrw] * G[1], temp_min_z, robotwb.max_force.z);
		if (vmc[id_swap[1]].param.trig_state == 4 && EN_TORT_LOAD_FORCE4) {//等待时load力
			robotwb.Leg[id_swap[1]].tar_force_dis_n.x = 0;
			robotwb.Leg[id_swap[1]].tar_force_dis_n.z = temp_min_z;
		}
		else if (vmc[id_swap[1]].param.trig_state == 5 && EN_TORT_LOAD_FORCE5) {//等待时load力
			robotwb.Leg[id_swap[1]].tar_force_dis_n.x = 0;
			robotwb.Leg[id_swap[1]].tar_force_dis_n.z = temp_min_z;
		}
		robotwb.Leg[id_swap[2]].tar_force_dis_n.x = limitw(F3c[Yrw] * G[2], -robotwb.max_force.x, robotwb.max_force.x);
		robotwb.Leg[id_swap[2]].tar_force_dis_n.y = F3c[Xrw] * G[2];
		robotwb.Leg[id_swap[2]].tar_force_dis_n.z = limitw(F3c[Zrw] * G[2], temp_min_z, robotwb.max_force.z);
		if (vmc[id_swap[2]].param.trig_state == 4 && EN_TORT_LOAD_FORCE4) {//等待时load力
			robotwb.Leg[id_swap[2]].tar_force_dis_n.x = 0;
			robotwb.Leg[id_swap[2]].tar_force_dis_n.z = temp_min_z;
		}
		else if (vmc[id_swap[2]].param.trig_state == 5 && EN_TORT_LOAD_FORCE5) {//等待时load力
			robotwb.Leg[id_swap[2]].tar_force_dis_n.x = 0;
			robotwb.Leg[id_swap[2]].tar_force_dis_n.z = temp_min_z;
		}
		robotwb.Leg[id_swap[3]].tar_force_dis_n.x = limitw(F4c[Yrw] * G[3], -robotwb.max_force.x, robotwb.max_force.x);
		robotwb.Leg[id_swap[3]].tar_force_dis_n.y = F4c[Xrw] * G[3];
		robotwb.Leg[id_swap[3]].tar_force_dis_n.z = limitw(F4c[Zrw] * G[3], temp_min_z, robotwb.max_force.z);
		if (vmc[id_swap[3]].param.trig_state == 4 && EN_TORT_LOAD_FORCE4) {//等待时load力
			robotwb.Leg[id_swap[3]].tar_force_dis_n.x = 0;
			robotwb.Leg[id_swap[3]].tar_force_dis_n.z = temp_min_z;
		}
		else if (vmc[id_swap[3]].param.trig_state == 5 && EN_TORT_LOAD_FORCE5) {//等待时load力
			robotwb.Leg[id_swap[3]].tar_force_dis_n.x = 0;
			robotwb.Leg[id_swap[3]].tar_force_dis_n.z = temp_min_z;
		}
	}
	else//站立  good-------------------------------------------------------------------------------------------
	{
#if 1//力分配结果最终输出 最终按实际着地情况
		G[0] = robotwb.Leg[id_swap[0]].is_ground*stand_force_enable_flag[4];
		G[1] = robotwb.Leg[id_swap[1]].is_ground*stand_force_enable_flag[4];
		G[2] = robotwb.Leg[id_swap[2]].is_ground*stand_force_enable_flag[4];
		G[3] = robotwb.Leg[id_swap[3]].is_ground*stand_force_enable_flag[4];
#endif

		robotwb.Leg[id_swap[0]].tar_force_dis_n.x = limitw(F1c[Yrw] * G[0], -robotwb.max_force.x, robotwb.max_force.x);
		robotwb.Leg[id_swap[0]].tar_force_dis_n.y = F1c[Xrw] * G[0];
		robotwb.Leg[id_swap[0]].tar_force_dis_n.z = limitw(F1c[Zrw] * G[0], temp_min_z, robotwb.max_force.z);//站立最小为load力

		robotwb.Leg[id_swap[1]].tar_force_dis_n.x = limitw(F2c[Yrw] * G[1], -robotwb.max_force.x, robotwb.max_force.x);
		robotwb.Leg[id_swap[1]].tar_force_dis_n.y = F2c[Xrw] * G[1];
		robotwb.Leg[id_swap[1]].tar_force_dis_n.z = limitw(F2c[Zrw] * G[1], temp_min_z, robotwb.max_force.z);

		robotwb.Leg[id_swap[2]].tar_force_dis_n.x = limitw(F3c[Yrw] * G[2], -robotwb.max_force.x, robotwb.max_force.x);
		robotwb.Leg[id_swap[2]].tar_force_dis_n.y = F3c[Xrw] * G[2];
		robotwb.Leg[id_swap[2]].tar_force_dis_n.z = limitw(F3c[Zrw] * G[2], temp_min_z, robotwb.max_force.z);

		robotwb.Leg[id_swap[3]].tar_force_dis_n.x = limitw(F4c[Yrw] * G[3], -robotwb.max_force.x, robotwb.max_force.x);
		robotwb.Leg[id_swap[3]].tar_force_dis_n.y = F4c[Xrw] * G[3];
		robotwb.Leg[id_swap[3]].tar_force_dis_n.z = limitw(F4c[Zrw] * G[3], temp_min_z, robotwb.max_force.z);
	}

	//-----------------------滤波
	static char g_reg[4] = { 0 };
	static char ff_reg[4] = { 0 };
	static float flt_w[4] = { 0 };
#if !USE_QP
	for (i = 0; i < 4; i++) {
		if (vmc_all.gait_mode == TROT) {
			if (G[i] == 0) {//离地后滤波器复位WS
				robotwb.Leg[i].tar_force_dis_n_reg.x = 0;
				robotwb.Leg[i].tar_force_dis_n_reg.y = 0;
				robotwb.Leg[i].tar_force_dis_n_reg.z = 0;
				//robotwb.Leg[i].tar_force_dis_n_reg.= robotwb.Leg[i].tar_force_dis_n;
				flt_w[i] = 0;
			}
			else {
				flt_w[i] += flt_t_dt * 2 * 0.005;
				//flt_w[i]=GRF_OUT_FLT;
				flt_w[i] = limitw(flt_w[i], 0.2, GRF_OUT_FLT);//解决抖腿

				robotwb.Leg[i].tar_force_dis_n.x = robotwb.Leg[i].tar_force_dis_n.x*flt_w[i] + robotwb.Leg[i].tar_force_dis_n_reg.x*(1 - flt_w[i]);
				robotwb.Leg[i].tar_force_dis_n.y = 0;
				robotwb.Leg[i].tar_force_dis_n.z = robotwb.Leg[i].tar_force_dis_n.z*flt_w[i] + robotwb.Leg[i].tar_force_dis_n_reg.z*(1 - flt_w[i]);
				robotwb.Leg[i].tar_force_dis_n_reg = robotwb.Leg[i].tar_force_dis_n;
			}
		}
		else {//STAND		
			if (G[i] == 0) {//离地后滤波器复位WS
				robotwb.Leg[i].tar_force_dis_n_reg.x = 0;
				robotwb.Leg[i].tar_force_dis_n_reg.y = 0;
				robotwb.Leg[i].tar_force_dis_n_reg.z = 0;
				//robotwb.Leg[i].tar_force_dis_n_reg.= robotwb.Leg[i].tar_force_dis_n;
				flt_w[i] = 0;
			}
			else
			{
				flt_w[i] += flt_t_dt * 0.005;
				flt_w[i] = limitw(flt_w[i], 0.15, GRF_OUT_FLT);//解决抖腿			

				robotwb.Leg[i].tar_force_dis_n.x = robotwb.Leg[i].tar_force_dis_n.x*flt_w[i] + robotwb.Leg[i].tar_force_dis_n_reg.x*(1 - flt_w[i]);
				robotwb.Leg[i].tar_force_dis_n.y = 0;
				robotwb.Leg[i].tar_force_dis_n.z = robotwb.Leg[i].tar_force_dis_n.z*flt_w[i] + robotwb.Leg[i].tar_force_dis_n_reg.z*(1 - flt_w[i]);
				robotwb.Leg[i].tar_force_dis_n_reg = robotwb.Leg[i].tar_force_dis_n;
			}
		}
	}

	for (i = 0; i < 4; i++) {
		g_reg[i] = vmc[i].ground;
		ff_reg[i] = stand_force_enable_flag[i];
	}
#endif
	//-----------------------摩擦圆锥
#if !USE_QP
			float force_z_mu_use = 0;
			for (i = 0; i < 4; i++) {
		#if 1
				force_z_mu_use = robotwb.Leg[i].tar_force_dis_n.z;//使用设定 good 一点
		#else
				force_z_mu_use = robotwb.Leg[i].force_est_n_output.z;//使用估计
		#endif
		#if 0//带地形
				robotwb.Leg[i].tar_force_dis_n.x = limitw(
					robotwb.Leg[i].tar_force_dis_n.x,
					-force_z_mu_use * 0.707*vmc_robot_p.ground_mu*(cosdw(fabs(vmc_all.ground_att_cmd[PITr] * 2))),
					force_z_mu_use*0.707*vmc_robot_p.ground_mu*(cosdw(fabs(vmc_all.ground_att_cmd[PITr] * 2))));

				robotwb.Leg[i].tar_force_dis_n.y = limitw(
					robotwb.Leg[i].tar_force_dis_n.y,
					-force_z_mu_use * 0.707*vmc_robot_p.ground_mu*(cosdw(fabs(vmc_all.ground_att_cmd[ROLr] * 2))),
					force_z_mu_use*0.707*vmc_robot_p.ground_mu*(cosdw(fabs(vmc_all.ground_att_cmd[ROLr] * 2))));
		#else
				robotwb.Leg[i].tar_force_dis_n.x = limitw(
					robotwb.Leg[i].tar_force_dis_n.x,
					-force_z_mu_use * 0.707*vmc_robot_p.ground_mu,
					force_z_mu_use*0.707*vmc_robot_p.ground_mu);

				robotwb.Leg[i].tar_force_dis_n.y = limitw(
					robotwb.Leg[i].tar_force_dis_n.y,
					-force_z_mu_use * 0.707*vmc_robot_p.ground_mu,
					force_z_mu_use*0.707*vmc_robot_p.ground_mu);
		#endif
			}
#endif

#if USE_QP&&0
	QP_Dis_Force();
	for(i=0;i<4;i++)
		robotwb.Leg[i].tar_force_dis_n=robotwb.Leg[i].tar_force_dis_n_qp;
#endif
}

void force_dis_n_side(void){
	int i = 0;
	float temp_min_z = pos_force_p.load_fz;
	float roll_fz = robotwb.exp_torque.x / Www / 2;
	//printf("ss\n");
	robotwb.Leg[0].tar_force_dis_n.x = robotwb.exp_force_b[F].x / 4 + robotwb.Leg[0].F_n_ff.x;
	robotwb.Leg[0].tar_force_dis_n.y = 0;
	robotwb.Leg[0].tar_force_dis_n.z = limitw(robotwb.exp_force_b[F].z / 2 - roll_fz / 2
		+ robotwb.Leg[0].F_n_ff.z, temp_min_z, robotwb.max_force.z);//站立最小为load力

	robotwb.Leg[2].tar_force_dis_n.x = robotwb.exp_force_b[F].x / 4 + robotwb.Leg[2].F_n_ff.x;
	robotwb.Leg[2].tar_force_dis_n.y = 0;
	robotwb.Leg[2].tar_force_dis_n.z = limitw(robotwb.exp_force_b[F].z / 2 + roll_fz / 2
		+ robotwb.Leg[2].F_n_ff.z, temp_min_z, robotwb.max_force.z);//站立最小为load力

	robotwb.Leg[1].tar_force_dis_n.x = robotwb.exp_force_b[B].x / 4 + robotwb.Leg[1].F_n_ff.x;
	robotwb.Leg[1].tar_force_dis_n.y = 0;
	robotwb.Leg[1].tar_force_dis_n.z = limitw(robotwb.exp_force_b[B].z / 2 - roll_fz / 2
		+ robotwb.Leg[1].F_n_ff.z, temp_min_z, robotwb.max_force.z);//站立最小为load力

	robotwb.Leg[3].tar_force_dis_n.x = robotwb.exp_force_b[B].x / 4 + robotwb.Leg[3].F_n_ff.x;
	robotwb.Leg[3].tar_force_dis_n.y = 0;
	robotwb.Leg[3].tar_force_dis_n.z = limitw(robotwb.exp_force_b[B].z / 2 + roll_fz / 2
		+ robotwb.Leg[3].F_n_ff.z, temp_min_z, robotwb.max_force.z);//站立最小为load力


	//-----------------------摩擦圆锥
#if !USE_QP
	float force_z_mu_use = 0;
	for (i = 0; i < 4; i++) {
#if 1
		force_z_mu_use = robotwb.Leg[i].tar_force_dis_n.z;//使用设定 good 一点
#else
		force_z_mu_use = robotwb.Leg[i].force_est_n_output.z;//使用估计
#endif
#if 0//带地形
		robotwb.Leg[i].tar_force_dis_n.x = limitw(
			robotwb.Leg[i].tar_force_dis_n.x,
			-force_z_mu_use * 0.707*vmc_robot_p.ground_mu*(cosdw(fabs(vmc_all.ground_att_cmd[PITr] * 2))),
			force_z_mu_use*0.707*vmc_robot_p.ground_mu*(cosdw(fabs(vmc_all.ground_att_cmd[PITr] * 2))));

		robotwb.Leg[i].tar_force_dis_n.y = limitw(
			robotwb.Leg[i].tar_force_dis_n.y,
			-force_z_mu_use * 0.707*vmc_robot_p.ground_mu*(cosdw(fabs(vmc_all.ground_att_cmd[ROLr] * 2))),
			force_z_mu_use*0.707*vmc_robot_p.ground_mu*(cosdw(fabs(vmc_all.ground_att_cmd[ROLr] * 2))));
#else
		robotwb.Leg[i].tar_force_dis_n.x = limitw(
			robotwb.Leg[i].tar_force_dis_n.x,
			-force_z_mu_use * 0.707*vmc_robot_p.ground_mu,
			force_z_mu_use*0.707*vmc_robot_p.ground_mu);

		robotwb.Leg[i].tar_force_dis_n.y = limitw(
			robotwb.Leg[i].tar_force_dis_n.y,
			-force_z_mu_use * 0.707*vmc_robot_p.ground_mu,
			force_z_mu_use*0.707*vmc_robot_p.ground_mu);
#endif
	}
#endif
}


void reset_servo_interge(void)
{
	robotwb.exp_force.x = robotwb.exp_force.y = robotwb.exp_force.z = 0;
	robotwb.exp_torque.x = robotwb.exp_torque.y = robotwb.exp_torque.z = 0;
	robotwb.exp_torque_i.x = robotwb.exp_torque_i.y = robotwb.exp_torque_i.z = 0;
	robotwb.exp_force_i.x = robotwb.exp_force_i.y = robotwb.exp_force_i.z = 0;
}



void force_control_and_dis_trot_release(float dt)
{
	int i = 0;
	float q_err[3] = { 0 };
	static char trig_state_reg[4] = { 0 };
	char pos_force_enable[3] = { 1,1,1 };
	Vect3 Vect3_zero;
	Vect3_zero.x = Vect3_zero.y = Vect3_zero.z = 0;
	Vect3 force_imp_spd_n = Vect3_zero;
	Vect3 force_imp_spd_g = Vect3_zero;
	Vect3 force_imp_spd_n_yaw = Vect3_zero;
	Vect3 force_imp_spd_h = Vect3_zero;
	Vect3 epos_h_next = Vect3_zero;
	Vect3 err_force_n = Vect3_zero, err_force_h = Vect3_zero;
	Vect3 tar_force_dis_n = Vect3_zero;
	float pos_taod[2] = { 0,0 };
	float fb_force_taod[2] = { 0,0 };
	float ff_force_taod[2] = { 0,0 };
	float dt_scale = 1;
	char diag_leg1[4] = { 2,3,0,1 };
	char diag_leg2[4] = { 1,0,3,2 };
	float force_z_mu_use = 0;
	static char ground_reg[4] = { 0 };

	for (i = 0; i < 4; i++) {

		pos_force_enable[0] = 1;

		if (ground_reg[i] != vmc[i].ground)
			reset_tar_pos(i);

		if (stand_force_enable_flag[4]//全局力控符号
			&& (vmc[i].ground || vmc[i].param.trig_state >= 4)
			&& USE_FPOS_CONTROL
			&&pos_force_p.en_force_control_cal
			&& !TEST_TROT_SW
			) {
			pos_force_enable[0] = 0;
		}
		else if (vmc[i].param.trig_state >= 1 && vmc[i].param.trig_state <= 3 && SWING_USE_SPD_MODE)//<---------------------------------位置模式 空中模式 摆动 等待状态4有力控
		{
			force_imp_spd_h.x = (vmc[i].param.tar_epos_h.x - vmc[i].tar_epos_h.x) / dt ;
			force_imp_spd_h.y = 0;
			force_imp_spd_h.z = (vmc[i].param.tar_epos_h.z - vmc[i].tar_epos_h.z) / dt ;

			force_imp_spd_h.x = limitw(force_imp_spd_h.x, -1.8, 1.8);
			force_imp_spd_h.y = 0;
			force_imp_spd_h.z = limitw(force_imp_spd_h.z, -1.8, 1.8);

			robotwb.Leg[i].force_imp_spd_h = force_imp_spd_h;//record			

			vmc[i].param.tar_epos_h.x -= force_imp_spd_h.x*dt;
			vmc[i].param.tar_epos_h.z -= force_imp_spd_h.z*dt;

			epos_h_next.x = vmc[i].param.tar_epos_h.x = limitw(vmc[i].param.tar_epos_h.x, MIN_X, MAX_X);//位置限制幅度
			epos_h_next.y = 0;
			epos_h_next.z = vmc[i].param.tar_epos_h.z = limitw(vmc[i].param.tar_epos_h.z, MAX_Z, MIN_Z);

			inv_KI(i, epos_h_next, &vmc[i].tar_sita1, &vmc[i].tar_sita2);
		}
		else if (gait_ww.state_gait > 2 && 1)//WS
		{//防止别处赋值造成阶跃
			vmc[i].param.tar_epos_h = vmc[i].tar_epos_h;
			robotwb.Leg[i].tar_epos_h.x = vmc[i].param.tar_epos_h.x;
			robotwb.Leg[i].tar_epos_h.y = vmc[i].param.tar_epos_h.y;
			robotwb.Leg[i].tar_epos_h.z = vmc[i].param.tar_epos_h.z;
		}

		//----------------------------------------记录缓存
		vmc[i].tar_epos_h_reg = vmc[i].tar_epos_h;//微分规划足端产生期望速度
		ground_reg[i] = vmc[i].ground;
		trig_state_reg[i] = vmc[i].param.trig_state;


		//************************************************************************************************************
		//**************************************************角度控制**************************************************
		//************************************************************************************************************
		if (gait_ww.state_gait > 2) {//非初始化下 限制位置期望范围 步态摆动
			vmc[i].tar_sita1 = limitw(vmc[i].tar_sita1, -robotwb.Leg[i].limit_sita[0], 90 - robotwb.Leg[i].limit_sita[1]);
			vmc[i].tar_sita2 = limitw(vmc[i].tar_sita2, 90 + robotwb.Leg[i].limit_sita[1], 180 + robotwb.Leg[i].limit_sita[0]);
		}
		robotwb.Leg[i].tar_sita[0] = vmc[i].tar_sita1;//robot tar 位置闭环期望赋值处
		robotwb.Leg[i].tar_sita[1] = vmc[i].tar_sita2;

		robotwb.Leg[i].err_sita[0] = To_180_degreesw(robotwb.Leg[i].tar_sita[0] - robotwb.Leg[i].sita[0]);
		robotwb.Leg[i].err_sita[1] = To_180_degreesw(robotwb.Leg[i].tar_sita[1] - robotwb.Leg[i].sita[1]);

		q_err[0] = limitw(To_180_degreesw(robotwb.Leg[i].tar_sita[0] - robotwb.Leg[i].sita[0]), -66, 66);
		q_err[1] = limitw(To_180_degreesw(robotwb.Leg[i].tar_sita[1] - robotwb.Leg[i].sita[1]), -66, 66);


#if !defined(CAN_ANL_MIT_MODE)
		pos_taod[0] = q_err[0] * robotwb.Leg[i].q_pid.kp * 10 / 1000. -
			limitw(robotwb.Leg[i].sita_d[0], -3500, 3500)*robotwb.Leg[i].q_pid.kd*dt_scale * 10 / 1000. +
			robotwb.Leg[i].tao_q_i[0];

		pos_taod[1] = q_err[1] * robotwb.Leg[i].q_pid.kp * 10 / 1000. -
			limitw(robotwb.Leg[i].sita_d[1], -3500, 3500)*robotwb.Leg[i].q_pid.kd*dt_scale * 10 / 1000. +
			robotwb.Leg[i].tao_q_i[1];

		pos_taod[0] += limitw(vmc[i].param.spd_dj[0] * robotwb.Leg[i].q_pid.vff* dt, -robotwb.Leg[i].limit_tao[0] * 0.2, robotwb.Leg[i].limit_tao[0] * 0.2);
		pos_taod[1] += limitw(vmc[i].param.spd_dj[1] * robotwb.Leg[i].q_pid.vff* dt, -robotwb.Leg[i].limit_tao[1] * 0.2, robotwb.Leg[i].limit_tao[1] * 0.2);

		robotwb.Leg[i].pos_taod[0] = pos_taod[0];//record
		robotwb.Leg[i].pos_taod[1] = pos_taod[1];//record
#else
		pos_taod[0] = robotwb.Leg[i].tao_q_i[0] * EN_Q_I_MIT_MODE;
		pos_taod[1] = robotwb.Leg[i].tao_q_i[1] * EN_Q_I_MIT_MODE;

		pos_taod[0] += limitw(vmc[i].param.spd_dj[0] * robotwb.Leg[i].q_pid.vff* dt, -robotwb.Leg[i].limit_tao[0] * 0.2, robotwb.Leg[i].limit_tao[0] * 0.2);
		pos_taod[1] += limitw(vmc[i].param.spd_dj[1] * robotwb.Leg[i].q_pid.vff* dt, -robotwb.Leg[i].limit_tao[1] * 0.2, robotwb.Leg[i].limit_tao[1] * 0.2);
#endif

		//************************************************************************************************************
		//***********************************************力Trot***************************************************
		//************************************************************************************************************

		if (stand_force_enable_flag[4] &&
			vmc[i].ground&&
			USE_FPOS_CONTROL&&
			pos_force_p.en_force_control_cal && !TEST_TROT_SW) {

			tar_force_dis_n = robotwb.Leg[i].tar_force_dis_n;

			if (vmc[i].is_touch == 0 && 0
				) {
				tar_force_dis_n.x *= 0;
				tar_force_dis_n.z *= 0;
			}
			tar_force_dis_n.y = 0;

			force_n_to_bw(tar_force_dis_n, &robotwb.Leg[i].tar_force_h);

			force_to_tao(i, dt);
			ff_force_taod[0] = robotwb.Leg[i].taod[0];
			ff_force_taod[1] = robotwb.Leg[i].taod[1];
		}
		else {
			fb_force_taod[0] = fb_force_taod[1] = 0;
			ff_force_taod[0] = ff_force_taod[1] = 0;
		}

		robotwb.Leg[i].ff_force_taod[0] = ff_force_taod[0];//record
		robotwb.Leg[i].ff_force_taod[1] = ff_force_taod[1];//record		

//************************************************************************************************************
//*************************************************输出合成***************************************************
//************************************************************************************************************
		if (gait_ww.state_gait <= 2) {//初始化或摆动仅角度模式
			robotwb.Leg[i].taod[0] = pos_taod[0] * pos_force_enable[0];
			robotwb.Leg[i].taod[1] = pos_taod[1] * pos_force_enable[0];
		}
		else {
			robotwb.Leg[i].taod[0] = pos_taod[0] * pos_force_enable[0] +
				ff_force_taod[0] * pos_force_enable[2] * pos_force_p.en_force_control_out;
			robotwb.Leg[i].taod[1] = pos_taod[1] * pos_force_enable[0] +
				ff_force_taod[1] * pos_force_enable[2] * pos_force_p.en_force_control_out;
		}
		set_motor_t(i);//CAN底层输出赋值
	}
}

void force_control_and_dis_stand_release(float dt)//
{
	int i = 0;
	float q_err[3] = { 0 };
	char pos_force_enable[3] = { 1,1,1 };
	Vect3 Vect3_zero;
	Vect3_zero.x = Vect3_zero.y = Vect3_zero.z = 0;
	Vect3 force_imp_spd_n = Vect3_zero;
	Vect3 force_imp_spd_n_yaw = Vect3_zero;
	Vect3 force_imp_spd_h = Vect3_zero;
	Vect3 force_imp_spd_g = Vect3_zero;
	Vect3 epos_h_next = Vect3_zero;
	Vect3 err_force_n = Vect3_zero, err_force_h = Vect3_zero;
	float pos_taod[2] = { 0,0 };
	float fb_force_taod[2] = { 0,0 };
	float ff_force_taod[2] = { 0,0 };
	float dt_scale = 1;
	Vect3 hip_rotate_spd_n, hip_rotate_spd_b;
	static char ground_reg[4] = { 0 };
	for (i = 0; i < 4; i++) {//站立
		pos_force_enable[0] = 1;
		if (ground_reg[i] != vmc[i].ground)
			reset_tar_pos(i);

		if (stand_force_enable_flag[4]//全局力控符号
			&& stand_force_enable_flag[i]//独立力控符号
			&& USE_FPOS_CONTROL
			&&pos_force_p.en_force_control_cal
			) {
			pos_force_enable[0] = 0;
		}
		else if (gait_ww.state_gait > 2 && (!stand_force_enable_flag[4] || !stand_force_enable_flag[i]))//<----位置模式 空中模式
		{

			force_imp_spd_h.x = limitw(vmc[i].param.tar_epos_h.x - vmc[i].tar_epos_h.x, -0.05, 0.05) / dt ;//复位末端期望位置  离地后如不是全离地则保证当前位置
			force_imp_spd_h.y = 0;
			force_imp_spd_h.z = limitw(vmc[i].param.tar_epos_h.z - vmc[i].tar_epos_h.z, -0.05, 0.05) / dt ;

			force_imp_spd_h.x = limitw(force_imp_spd_h.x, -0.8, 0.8);
			force_imp_spd_h.y = 0;
			force_imp_spd_h.z = limitw(force_imp_spd_h.z, -0.8, 0.8);
			END_POS force_imp_spd_ht;
			force_imp_spd_ht.x = force_imp_spd_h.x;
			force_imp_spd_ht.z = force_imp_spd_h.z;

			espd_to_qspd(&vmc[i], force_imp_spd_ht, vmc[i].param.spd_dj, dt);

			converV_n_to_bw(force_imp_spd_n, &force_imp_spd_h);

			robotwb.Leg[i].force_imp_spd_h = force_imp_spd_h;

			vmc[i].param.tar_epos_h.x -= force_imp_spd_h.x*dt;
			vmc[i].param.tar_epos_h.z -= force_imp_spd_h.z*dt;
			epos_h_next.x = vmc[i].param.tar_epos_h.x = limitw(vmc[i].param.tar_epos_h.x, MIN_X, MAX_X);//位置限制幅度
			epos_h_next.z = vmc[i].param.tar_epos_h.z = limitw(vmc[i].param.tar_epos_h.z, MAX_Z, MIN_Z);
			inv_KI(i, epos_h_next, &vmc[i].tar_sita1, &vmc[i].tar_sita2);
		}
		else if (gait_ww.state_gait > 2 && 1) {//防止别处赋值造成阶跃WS 
			vmc[i].tar_sita1 = vmc[i].sita1;
			vmc[i].tar_sita2 = vmc[i].sita2;
		}

		//----------------------------------------记录缓存
		vmc[i].tar_epos_h_reg = vmc[i].tar_epos_h;//微分规划足端产生期望速度
		ground_reg[i] = vmc[i].ground;


		//************************************************************************************************************
		//**************************************************角度控制**************************************************
		//************************************************************************************************************
		if (gait_ww.state_gait > 2) {//非初始化下 限制位置期望范围 步态摆动
			vmc[i].tar_sita1 = limitw(vmc[i].tar_sita1, -robotwb.Leg[i].limit_sita[0], 90 - robotwb.Leg[i].limit_sita[1]);
			vmc[i].tar_sita2 = limitw(vmc[i].tar_sita2, 90 + robotwb.Leg[i].limit_sita[1], 180 + robotwb.Leg[i].limit_sita[0]);
		}
		robotwb.Leg[i].tar_sita[0] = vmc[i].tar_sita1;//robot tar 位置闭环期望赋值处
		robotwb.Leg[i].tar_sita[1] = vmc[i].tar_sita2;

		robotwb.Leg[i].err_sita[0] = To_180_degreesw(robotwb.Leg[i].tar_sita[0] - robotwb.Leg[i].sita[0]);
		robotwb.Leg[i].err_sita[1] = To_180_degreesw(robotwb.Leg[i].tar_sita[1] - robotwb.Leg[i].sita[1]);

		q_err[0] = limitw(To_180_degreesw(robotwb.Leg[i].tar_sita[0] - robotwb.Leg[i].sita[0]), -66, 66);
		q_err[1] = limitw(To_180_degreesw(robotwb.Leg[i].tar_sita[1] - robotwb.Leg[i].sita[1]), -66, 66);


#if !defined(CAN_ANL_MIT_MODE)		
		pos_taod[0] = q_err[0] * robotwb.Leg[i].q_pid.kp * 10 / 1000. -
			limitw(robotwb.Leg[i].sita_d[0], -3500, 3500)*robotwb.Leg[i].q_pid.kd*dt_scale * 10 / 1000. +
			robotwb.Leg[i].tao_q_i[0];

		pos_taod[1] = q_err[1] * robotwb.Leg[i].q_pid.kp * 10 / 1000. -
			limitw(robotwb.Leg[i].sita_d[1], -3500, 3500)*robotwb.Leg[i].q_pid.kd*dt_scale * 10 / 1000. +
			robotwb.Leg[i].tao_q_i[1];

		pos_taod[0] += limitw(vmc[i].param.spd_dj[0] * robotwb.Leg[i].q_pid.vff* dt, -robotwb.Leg[i].limit_tao[0] * 0.2, robotwb.Leg[i].limit_tao[0] * 0.2);
		pos_taod[1] += limitw(vmc[i].param.spd_dj[1] * robotwb.Leg[i].q_pid.vff* dt, -robotwb.Leg[i].limit_tao[1] * 0.2, robotwb.Leg[i].limit_tao[1] * 0.2);

		robotwb.Leg[i].pos_taod[0] = pos_taod[0];//record
		robotwb.Leg[i].pos_taod[1] = pos_taod[1];//record
#else
		pos_taod[0] = robotwb.Leg[i].tao_q_i[0] * EN_Q_I_MIT_MODE;
		pos_taod[1] = robotwb.Leg[i].tao_q_i[1] * EN_Q_I_MIT_MODE;
#endif

		//************************************************************************************************************
		//**************************************************力控制*************************************************
		//************************************************************************************************************
		if (stand_force_enable_flag[4] && stand_force_enable_flag[i] &&//all enable
			USE_FPOS_CONTROL&&
			pos_force_p.en_force_control_cal) {

			if (vmc[i].ground == 0) {//非着地下 
				robotwb.Leg[i].tar_force_dis_n.x *= 0;
				robotwb.Leg[i].tar_force_dis_n.z *= 0;
			}
			robotwb.Leg[i].tar_force_dis_n.y = 0;

			force_n_to_bw(robotwb.Leg[i].tar_force_dis_n, &robotwb.Leg[i].tar_force_h);

			force_to_tao(i, dt);
			ff_force_taod[0] = robotwb.Leg[i].taod[0] * vmc[i].ground;
			ff_force_taod[1] = robotwb.Leg[i].taod[1] * vmc[i].ground;
		}
		else {
			ff_force_taod[0] = ff_force_taod[1] = 0;
		}

		robotwb.Leg[i].ff_force_taod[0] = ff_force_taod[0];//record
		robotwb.Leg[i].ff_force_taod[1] = ff_force_taod[1];//record		

//************************************************************************************************************
//*************************************************输出合成***************************************************
//************************************************************************************************************
		if (gait_ww.state_gait <= 2) {//初始化或摆动仅角度模式
			robotwb.Leg[i].taod[0] = pos_taod[0] * pos_force_enable[0];
			robotwb.Leg[i].taod[1] = pos_taod[1] * pos_force_enable[0];
		}
		else {
			robotwb.Leg[i].taod[0] = pos_taod[0] * pos_force_enable[0] +
				robotwb.Leg[i].w_force_taod[0] +
				ff_force_taod[0] * pos_force_enable[2] * pos_force_p.en_force_control_out;
			robotwb.Leg[i].taod[1] = pos_taod[1] * pos_force_enable[0] +
				robotwb.Leg[i].w_force_taod[1] +
				ff_force_taod[1] * pos_force_enable[2] * pos_force_p.en_force_control_out;
		}
		set_motor_t(i);//CAN底层输出赋值
	}
}
