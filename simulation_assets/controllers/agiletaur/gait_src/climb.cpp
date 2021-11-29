#include "gait_math.h"
#include "eso.h"
#include "locomotion_header.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

Gait_Climb gait_climb;

static END_POS end_pos_stand_h[4], end_pos_stand_n[4];
static char st_mode_reg = 0;

static float stand_gcheck_time[2] = { 0.01,0.05 };//TD  LF 着地时间判断与离地时间
static END_POS vmc_all_pos_n_air_st;
static END_POS vmc_all_spd_n_air_st;

void  Gait_Climb_Active(void)
{
	int i = 0;
	vmc_all.gait_mode = CLIMB;
	vmc_all.param.robot_mode = M_CLIMB;
	//state estimateor
	for (i = 0; i < 4; i++) {
		vmc[i].tar_sita1 = vmc[i].sita1;
		vmc[i].tar_sita2 = vmc[i].sita2;
		vmc[i].tar_sita3 = vmc[i].sita3;

		estimate_end_state_new(&vmc[i], 0.005);

		cal_jacobi_new(&vmc[i]);

		end_pos_stand_h[i] = vmc[i].epos;
		end_pos_stand_n[i] = vmc[i].epos_n;

		reset_tar_pos(i);//初始化力控导纳位置为当前
	}

	for (i = 0; i < 3; i++) {
		vmc_all.param.test_att_flt[i] = 0;
		vmc_all.param.test_pos_flt[i] = 0;
	}

	vmc_all_pos_n_air_st.x = (vmc[0].epos_n.x + vmc[1].epos_n.x + vmc[2].epos_n.x + vmc[3].epos_n.x) / 4;

	vmc_all_pos_n_air_st.z = -(vmc[0].epos_n.z + vmc[1].epos_n.z + vmc[2].epos_n.z + vmc[3].epos_n.z) / 4;
	vmc_all_spd_n_air_st.z = (vmc[0].spd_n.z + vmc[1].spd_n.z + vmc[2].spd_n.z + vmc[3].spd_n.z) / 4;

	vmc_all.tar_pos.z = vmc_all.param.test_pos_flt[Zr] = fabs(vmc_all_pos_n_air_st.z);

	vmc_all.tar_att[YAWr] = vmc_all.att_ctrl[YAWr];
	st_mode_reg = vmc_all.gait_mode;


	if (stand_force_enable_flag[4] == 0) {
		stand_force_enable_flag[0] = 0;
		stand_force_enable_flag[1] = 0;
		stand_force_enable_flag[2] = 0;
		stand_force_enable_flag[3] = 0;
		stand_force_enable_flag[4] = 0;
	}//清除力控标志位

	gait_climb.state = 0;
	gait_climb.timer[0] = gait_climb.timer[1] = gait_climb.timer[2] = 0;
	gait_climb.jump_height_off = 0.1;
	gait_climb.jump_height = vmc_all.pos_n.z + gait_climb.jump_height_off;
	gait_climb.jump_w = 0 / 57.3;
	gait_climb.kp[Xr] = 70;

	gait_climb.fp[Xr] = 6;
	gait_climb.fp[Zr] = 15;

	gait_climb.lf_time = 0.25;//s
	gait_climb.couch_time = gait_climb.lf_time*0.618;//s
	gait_climb.jump_spdx = 0.1;

	robotwb.cog_F_ff.x = 0;
	robotwb.cog_F_ff.z = 0;

	robotwb.cog_T_ff.z = 0;

	for (i = 0; i < 4; i++) {
		robotwb.Leg[i].F_n_ff.x = robotwb.Leg[i].F_n_ff.y = robotwb.Leg[i].F_n_ff.z = 0;
		robotwb.Leg[i].w_force_taod[0] = robotwb.Leg[i].w_force_taod[1] = 0;
	}

	gait_climb.force_control_mode = 0;//整体力控 1->前后独立力控
#if RUN_WEBOTS
	printf("Climb Gait Initing!!!!!!!\n");
#endif
}


void  Gait_Climb_Update(float dt)
{
	static char init[4], init_lisence = 0, state, cnt_time_change, switch_flag;
	static float time_delay, yaw_force;
	static float timer_task[10];
	char i, sel;
	float test_epos_b[3], test_epos_leg[3];
	static END_POS end_pos_n[4], end_pos_n_use[4];//ĩ�˹̶���ȫ������ϵ�µ�λ��
	static float cnt_mode_can_switch = 0, mode_en_switch_flag = 0;
	static float att_test_use[3];
	static char mode_reg;
	float att_off[2];
	float sincos_temp[3] = { 0 };
	//state estimateor
	static float ground_timer[4][2] = { 0 };
	static float tar_air_z = 0;
	static float tar_air_x = 0;
	static float com_x_reg = 0, com_x_off = 0;
	float gain_w = 0.01;
	vmc_all.ground_num = 0;
	for (i = 0; i < 4; i++) {

		vmc[i].ground = vmc[i].is_touch;

		if (vmc[i].ground)
			vmc_all.ground_num++;

		if (st_mode_reg != vmc_all.gait_mode) {//模式切换下赋为当前值
			end_pos_stand_h[i] = vmc[i].epos;
			end_pos_stand_n[i] = vmc[i].epos_n;
			vmc_all.param.test_pos_flt[Zr] = -vmc_all.pos_n.z;
			vmc_all.param.test_pos_flt[Xr] = vmc_all.param.test_pos_flt[Yr] = 0;
		}
	}
	//gait_climb.jump_x_dis = ocu.rc_spd_w[Xr];

	//空中模式下的状态估计
	vmc_all_pos_n_air_st.x = (vmc[0].epos_n.x + vmc[1].epos_n.x + vmc[2].epos_n.x + vmc[3].epos_n.x) / 4;
	vmc_all_pos_n_air_st.z = -(vmc[0].epos_n.z + vmc[1].epos_n.z + vmc[2].epos_n.z + vmc[3].epos_n.z) / 4;

	vmc_all_spd_n_air_st.x = (vmc[0].spd_n.x + vmc[1].spd_n.x + vmc[2].spd_n.x + vmc[3].spd_n.x) / 4;
	vmc_all_spd_n_air_st.z = (vmc[0].spd_n.z + vmc[1].spd_n.z + vmc[2].spd_n.z + vmc[3].spd_n.z) / 4;

	st_mode_reg = vmc_all.gait_mode;

	static float en_force_timer[2] = { 0 };
#if STAND_GROUND_CHECK_TEST//力控标志位赋值
	if ((vmc_all.ground_num >= 2
		|| (vmc[0].ground&&vmc[3].ground)
		|| (vmc[1].ground&&vmc[2].ground)
		) && !stand_force_enable_flag[4])//着地
		en_force_timer[0] += dt;
	else
		en_force_timer[0] = 0;

	if (vmc_all.ground_num <= 1 && stand_force_enable_flag[4])
		en_force_timer[1] += dt;
	else
		en_force_timer[1] = 0;
	//------------------------------------------------------------------------------
	if (en_force_timer[0] > stand_gcheck_time[0] - dt && !stand_force_enable_flag[4]) {//着地切换力控
		en_force_timer[0] = 0;
		reset_servo_interge();//清除积分
		for (i = 0; i < 4; i++) {//清除力复位力控期望
			//robotwb.Leg[i].tar_force_dis_n.x = robotwb.Leg[i].tar_force_dis_n.z = 0;
			reset_tar_pos(i);
		}

		robotwb.exp_pos_n.x = vmc_all.tar_pos.x = 0;//复位力控期望
		robotwb.exp_pos_n.z = fabs(vmc_all_pos_n_air_st.z);
		tar_air_z = vmc_all.param.test_pos_flt[Zr] = -fabs(vmc_all_pos_n_air_st.z);

		stand_force_enable_flag[4] = 1;
#if RUN_WEBOTS
		printf(" Ground Mode!!\n");
#endif
	}
	else if (en_force_timer[1] > stand_gcheck_time[1] - dt && stand_force_enable_flag[4] == 1) {//离地AIR模式
		//sit_down_flag=0;
		en_force_timer[1] = 0;
		reset_servo_interge();//清除积分
		for (i = 0; i < 4; i++) {//清除力复位力控期望
			//robotwb.Leg[i].tar_force_dis_n.x = robotwb.Leg[i].tar_force_dis_n.z = 0;
			reset_tar_pos(i);
		}
		robotwb.exp_pos_n.x = 0;
		robotwb.exp_pos_n.z = fabs(vmc_all_pos_n_air_st.z);
		tar_air_z = vmc_all.param.test_pos_flt[Zr] = -fabs(vmc_all_pos_n_air_st.z);//足端位置

		stand_force_enable_flag[4] = 0;
#if RUN_WEBOTS
		printf(" Air Mode!!\n");
#endif
	}
#endif	

	gait_climb.jump_trig = 1;
	switch (gait_climb.state)
	{
	case 0:
		if (gait_climb.jump_trig&&fabs(vmc_all.att[ROLr]) < 5) {//前腿
			gait_climb.jump_trig = 0;
#if 1
			gait_climb.jump_phase = F;
			gait_climb.jump_phase_other = B;
#else
			gait_climb.jump_phase = B;
			gait_climb.jump_phase_other = F;
#endif
			gait_climb.state++;
			gait_climb.timer[0] = gait_climb.timer[1] = gait_climb.timer[2] = 0;

#if RUN_WEBOTS
			printf("CLIMB Planning...\n");
#endif
		}
		break;
	case 1:
		gait_climb.timer[0] += dt;


		if (gait_climb.timer[0] >0.015&&1) {
			gait_climb.timer[0] = 0;

			gait_climb.force_control_mode = 1;
			gait_climb.state++;

			gait_climb.st_height = fabs(vmc_all.pos_n_b[gait_climb.jump_phase].z);
			gait_climb.couch_height = fabs(MIN_Z)*1.3;

			gait_climb.up_stair = 0;
			if (vmc_all.ground_att_est[PITr] > 3) {
				gait_climb.up_stair = 1;
				//gait_climb.lf_time = 0.15;
			}

			if (gait_climb.jump_phase == F)
				gait_climb.jump_height = vmc_all.pos_n.z + gait_climb.jump_height_off;
			else if(gait_climb.up_stair&&gait_climb.jump_phase == B)
				gait_climb.jump_height = vmc_all.pos_n.z + gait_climb.jump_height_off*1.5;

			robotwb.exp_pos_n_b[gait_climb.jump_phase].x = vmc_all.pos_n_b[gait_climb.jump_phase].x;
			robotwb.exp_pos_n_b[gait_climb.jump_phase].z = fabs(vmc_all.pos_n_b[gait_climb.jump_phase].z);
			robotwb.exp_pos_n_b[gait_climb.jump_phase_other].x = vmc_all.pos_n_b[gait_climb.jump_phase_other].x;
			robotwb.exp_pos_n_b[gait_climb.jump_phase_other].z = fabs(vmc_all.pos_n_b[gait_climb.jump_phase_other].z)*0.9;
#if RUN_WEBOTS
			printf("CLIMB Start & Couch up_stair=%d!!\n", gait_climb.up_stair);
#endif
		}
		break;
	case 2://下蹲
		gait_climb.timer[0] += dt;
		sincos_temp[0] = limitw(gait_climb.timer[0] / gait_climb.couch_time, 0, 1) * 90;

		robotwb.exp_pos_n_b[gait_climb.jump_phase].x = 0;
		robotwb.exp_pos_n_b[gait_climb.jump_phase].z = gait_climb.st_height - (gait_climb.st_height - gait_climb.couch_height)*sind(sincos_temp[0]);
		
		if (gait_climb.timer[0] >= gait_climb.couch_time &&1 ) {
			gait_climb.timer[0] = 0;
			gait_climb.state++;

			gait_climb.ch_height = fabs(vmc_all.pos_n_b[gait_climb.jump_phase].z);
			gait_climb.jump_power = (gait_climb.jump_height - gait_climb.ch_height)* Mw * gw/2;

			gait_climb.jump_avg_F[Xr] = gait_climb.fp[Xr] * gait_climb.jump_spdx* Mw * gw ;
			if (gait_climb.up_stair&&gait_climb.jump_phase==B)
				gait_climb.jump_avg_F[Zr] = gait_climb.fp[Zr] * gait_climb.jump_power*2/ cosd(vmc_all.ground_att_est[PITr]);
			else
				gait_climb.jump_avg_F[Zr] = gait_climb.fp[Zr] * gait_climb.jump_power*cosd(vmc_all.ground_att_est[PITr]);

			//printf("%f %f %f\n", gait_climb.kp[Zr], gait_climb.jump_avg_F[Zr], gait_climb.jump_power);
#if RUN_WEBOTS
			printf(" CLIMB Finish & Start to Jump G_att=%f!!\n", vmc_all.ground_att_est[PITr]);
#endif
			com_x_reg = vmc_all.pos_n_b[gait_climb.jump_phase_other].x;
		}
		break;

	case 3://蹬腿
		gait_climb.timer[0] += dt;
		gait_climb.timer[1] += dt;

		sincos_temp[0] = limitw(gait_climb.timer[0] / gait_climb.lf_time, 0, 1) * 90;
		sincos_temp[1] = limitw(gait_climb.timer[1] / gait_climb.lf_time, 0, 1) * 180;

		//robotwb.exp_pos_n_b[gait_climb.jump_phase].x = 0;//不加不行
		//robotwb.exp_pos_n_b[gait_climb.jump_phase].x = vmc_all.pos_n_b[gait_climb.jump_phase].x;
		robotwb.exp_pos_n_b[gait_climb.jump_phase].z = gait_climb.ch_height + (gait_climb.jump_height - gait_climb.ch_height)*sind(sincos_temp[0]);//

		if (gait_climb.jump_phase == F) {//蹬地力
			robotwb.Leg[0].F_n_ff.x = gait_climb.jump_avg_F[Xr] * sind(sincos_temp[1]) / 2 * 0;
			robotwb.Leg[0].F_n_ff.z = gait_climb.jump_avg_F[Zr] * sind(sincos_temp[1]) / 2;
			robotwb.Leg[2].F_n_ff.x = gait_climb.jump_avg_F[Xr] * sind(sincos_temp[1]) / 2 * 0;
			robotwb.Leg[2].F_n_ff.z = gait_climb.jump_avg_F[Zr] * sind(sincos_temp[1]) / 2;

			robotwb.Leg[1].w_force_taod[0] = vmc_all.att_rate[PITr] * -gain_w;
			robotwb.Leg[1].w_force_taod[1] = vmc_all.att_rate[PITr] * -gain_w;
			robotwb.Leg[3].w_force_taod[0] = vmc_all.att_rate[PITr] * -gain_w;
			robotwb.Leg[3].w_force_taod[1] = vmc_all.att_rate[PITr] * -gain_w;
		}
		else {
			robotwb.Leg[1].F_n_ff.x = gait_climb.jump_avg_F[Xr] * sind(sincos_temp[1]) / 2 * 0;
			robotwb.Leg[1].F_n_ff.z = gait_climb.jump_avg_F[Zr] * sind(sincos_temp[1]) / 2;
			robotwb.Leg[3].F_n_ff.x = gait_climb.jump_avg_F[Xr] * sind(sincos_temp[1]) / 2 * 0;
			robotwb.Leg[3].F_n_ff.z = gait_climb.jump_avg_F[Zr] * sind(sincos_temp[1]) / 2;

			robotwb.Leg[0].w_force_taod[0] = vmc_all.att_rate[PITr] * -gain_w;
			robotwb.Leg[0].w_force_taod[1] = vmc_all.att_rate[PITr] * -gain_w;
			robotwb.Leg[2].w_force_taod[0] = vmc_all.att_rate[PITr] * -gain_w;
			robotwb.Leg[2].w_force_taod[1] = vmc_all.att_rate[PITr] * -gain_w;
		}

		if (gait_climb.jump_phase == F && vmc[0].ground&&vmc[2].ground) {//计算离地速度
			gait_climb.lf_spd[Xr] = vmc_all.spd_n_b[gait_climb.jump_phase].x;
			gait_climb.lf_spd[Zr] = vmc_all.spd_n_b[gait_climb.jump_phase].z;
		}
		else if (gait_climb.jump_phase == B && vmc[1].ground&&vmc[3].ground) {
			gait_climb.lf_spd[Xr] = vmc_all.spd_n_b[gait_climb.jump_phase].x;
			gait_climb.lf_spd[Zr] = vmc_all.spd_n_b[gait_climb.jump_phase].z;
		}

		if (gait_climb.timer[0] >= gait_climb.lf_time //离地判断
			|| (vmc_all.ground_num <= 2 && gait_climb.timer[0] >= gait_climb.lf_time*0.3)
			) {
			gait_climb.timer[2] = gait_climb.timer[1] = gait_climb.timer[0] = 0;
			gait_climb.state++;
			if (gait_climb.jump_phase == F) 
				stand_force_enable_flag[0] = stand_force_enable_flag[2] = 0;
			else
				stand_force_enable_flag[1] = stand_force_enable_flag[3] = 0;

			gait_climb.fly_time = 2 * fabs(gait_climb.lf_spd[Zr]) / gw;//计算飞行时间
			gait_climb.fly_time_real = 0;

			tar_air_x = MAX_X*0.8;
			tar_air_z = -fabs(MIN_Z)*1.5;

			robotwb.exp_pos_n_b[gait_climb.jump_phase].x = tar_air_x;
			robotwb.exp_pos_n_b[gait_climb.jump_phase].z = -tar_air_z;

			com_x_off = vmc_all.pos_n_b[gait_climb.jump_phase_other].x - com_x_reg;//质心偏差
#if RUN_WEBOTS
			printf("fly_time=%f lf_spdx=%f lf_spdz=%f\n", gait_climb.fly_time, gait_climb.lf_spd[Xr], gait_climb.lf_spd[Zr]);
			printf(" CLIMB Jump Finish!!\n");
			printf("com_x_off=%f\n", com_x_off);
#endif
			for (i = 0; i < 4; i++)
				robotwb.Leg[i].F_n_ff.x = robotwb.Leg[i].F_n_ff.y = robotwb.Leg[i].F_n_ff.z = 0;
		}
		break;
	case 4://腾空-----------------------------------------
		if (gait_climb.jump_phase == F) {//落足点
			if (vmc_all.spd_n_b[gait_climb.jump_phase_other].x > 0 || 1)
				tar_air_x = vmc_all.spd_n_b[gait_climb.jump_phase_other].x * gait_climb.fly_time  * 1- com_x_off * 6;
			else
				tar_air_x = 0;

			if(gait_climb.up_stair)
				tar_air_x = vmc_all.spd_n_b[gait_climb.jump_phase_other].x * gait_climb.fly_time * 1 - com_x_off * 2;

			tar_air_x = limitw(tar_air_x, MIN_X*0.3, MAX_X);
		}
		else {
			if (vmc_all.spd_n_b[gait_climb.jump_phase_other].x > 0 || 1)
				tar_air_x = vmc_all.spd_n_b[gait_climb.jump_phase_other].x * gait_climb.fly_time  * 1- com_x_off * 6;
			else
				tar_air_x = 0;

			if (gait_climb.up_stair) {
				tar_air_x = vmc_all.spd_n_b[gait_climb.jump_phase_other].x * gait_climb.fly_time * 1 - com_x_off * 4;
				//tar_air_z = -fabs(MIN_Z)*1.3;
			}
			tar_air_x = limitw(tar_air_x, MIN_X, MAX_X*0.3);
		}
		tar_air_x *= cosd(vmc_all.ground_att_est[PITr]);
		

		gait_climb.timer[0] += dt;
		sincos_temp[0] = limitw(gait_climb.timer[0] / gait_climb.fly_time, 0, 1) * 180;//X力前馈

		robotwb.exp_pos_n_b[gait_climb.jump_phase_other].x = vmc_all.pos_n_b[gait_climb.jump_phase_other].x;

		if (gait_climb.jump_phase == F) {//支撑腿 控制
			robotwb.Leg[1].F_n_ff.x = gait_climb.jump_avg_F[Xr] * sind(sincos_temp[0]) / 2;
			robotwb.Leg[3].F_n_ff.x = gait_climb.jump_avg_F[Xr] * sind(sincos_temp[0]) / 2;

			robotwb.Leg[1].w_force_taod[0] = vmc_all.att_rate[PITr] * -gain_w;
			robotwb.Leg[1].w_force_taod[1] = vmc_all.att_rate[PITr] * -gain_w;
			robotwb.Leg[3].w_force_taod[0] = vmc_all.att_rate[PITr] * -gain_w;
			robotwb.Leg[3].w_force_taod[1] = vmc_all.att_rate[PITr] * -gain_w;

			robotwb.Leg[1].F_n_ff.x += gait_climb.kp[Xr] * (gait_climb.jump_spdx - vmc_all.spd_n_b[B].x);
			robotwb.Leg[3].F_n_ff.x += gait_climb.kp[Xr] * (gait_climb.jump_spdx - vmc_all.spd_n_b[B].x);
			if (gait_climb.up_stair) {
				robotwb.Leg[1].F_n_ff.x /= cosd(vmc_all.ground_att_est[PITr]);
				robotwb.Leg[3].F_n_ff.x /= cosd(vmc_all.ground_att_est[PITr]);
			}
		}
		else {
			robotwb.Leg[0].F_n_ff.x = gait_climb.jump_avg_F[Xr] * sind(sincos_temp[0]) / 2;
			robotwb.Leg[2].F_n_ff.x = gait_climb.jump_avg_F[Xr] * sind(sincos_temp[0]) / 2;

			robotwb.Leg[0].w_force_taod[0] = vmc_all.att_rate[PITr] * -gain_w;
			robotwb.Leg[0].w_force_taod[1] = vmc_all.att_rate[PITr] * -gain_w;
			robotwb.Leg[2].w_force_taod[0] = vmc_all.att_rate[PITr] * -gain_w;
			robotwb.Leg[2].w_force_taod[1] = vmc_all.att_rate[PITr] * -gain_w;
			if (!gait_climb.up_stair) {
				robotwb.Leg[0].F_n_ff.x -= gait_climb.kp[Xr] * (gait_climb.jump_spdx - vmc_all.spd_n_b[F].x);
				robotwb.Leg[2].F_n_ff.x -= gait_climb.kp[Xr] * (gait_climb.jump_spdx - vmc_all.spd_n_b[F].x);
			}


		}

		vmc_all.pos_n.x = vmc_all_pos_n_air_st.x;
		vmc_all.pos_n.z = vmc_all_pos_n_air_st.z;
		vmc_all.spd_n.x = 0;
		vmc_all.spd_n.z = 0;

		if (vmc_all.ground_num <= 2)
			gait_climb.fly_time_real += dt;

		if (vmc_all.ground_num >= 4 && 1) 
			gait_climb.timer[1] += dt;
		else
			gait_climb.timer[1] = 0;

		//printf("%f\n", vmc_all.att_rate[PITr]);
		if(gait_climb.timer[1]> 0.025
			/*|| (gait_climb.timer[1]> gait_climb.fly_time*0.25&&
			((vmc_all.att_rate[PITr] <0&& gait_climb.jump_phase  ==F)||
				(vmc_all.att_rate[PITr] >0 && gait_climb.jump_phase ==B))
			)*/
			){
			gait_climb.timer[2] = gait_climb.timer[1] = gait_climb.timer[0] = 0;
			robotwb.exp_pos_n_b[gait_climb.jump_phase].x = vmc_all.pos_n_b[gait_climb.jump_phase].x;
			robotwb.exp_pos_n_b[gait_climb.jump_phase].z = fabs(vmc_all.pos_n_b[gait_climb.jump_phase].z);
			robotwb.exp_pos_n_b[gait_climb.jump_phase_other].x = vmc_all.pos_n_b[gait_climb.jump_phase_other].x;
			robotwb.exp_pos_n_b[gait_climb.jump_phase_other].z = fabs(vmc_all.pos_n_b[gait_climb.jump_phase_other].z);

			gait_climb.state++;

			for (i = 0; i < 4; i++) {
				robotwb.Leg[i].F_n_ff.x = robotwb.Leg[i].F_n_ff.y = robotwb.Leg[i].F_n_ff.z = 0;
				robotwb.Leg[i].w_force_taod[0] = robotwb.Leg[i].w_force_taod[1] = 0;
			}
#if RUN_WEBOTS
			printf(" Ground && Reheight fly_time=%f!!\n", gait_climb.fly_time_real);
#endif
			gait_climb.force_control_mode = 0;

			if (gait_climb.jump_phase == F) {
				gait_climb.jump_phase = B;
				gait_climb.jump_phase_other = F;
				stand_force_enable_flag[0] = stand_force_enable_flag[2] = 1;
			}
			else {
				gait_climb.jump_phase = F;
				gait_climb.jump_phase_other = B;
				stand_force_enable_flag[1] = stand_force_enable_flag[3] = 1;
			}

			robotwb.exp_pos_n.x=vmc_all.tar_pos.x = tar_air_x = 0;
			robotwb.exp_pos_n.z=vmc_all.tar_pos.z = 0.7*fabs(MAX_Z);
		}
		break;
	case 5:
		
		if (fabs(vmc_all.att[ROLr]) < 5
			&& fabs(fabs(vmc_all.tar_pos.z) - vmc_all.pos_n.z) < 0.02
			) gait_climb.timer[0] += dt; 
		else
			gait_climb.timer[0]=0; 

		if(gait_climb.timer[0] > 0.015)
		{
			gait_climb.timer[2] = gait_climb.timer[1] = gait_climb.timer[0] = 0;

			gait_climb.state =1;
			printf(" Climb Done!!\n");
		}
		break;

	default:
		break;
	}


	for (i = 0; i < 4; i++) {
		if (stand_force_enable_flag[4] == 0 || stand_force_enable_flag[i]==0)//空中位置模式式
		{
			//for (i = 0; i < 4; i++) {//期望末端位置
			end_pos_n[i].x = end_pos_stand_n[i].x + vmc_all.param.param_vmc.posz_idle[Xr];
			end_pos_n[i].y = end_pos_stand_n[i].y + vmc_all.param.param_vmc.posz_idle[Yr];
			end_pos_n[i].z = end_pos_stand_n[i].z;
	
			vmc_all.param.test_pos_flt[Xr] = tar_air_x;
			vmc_all.param.test_pos_flt[Xr] = LIMIT(vmc_all.param.test_pos_flt[Xr], MIN_X,MAX_X);

			vmc_all.param.test_pos_flt[Yr] += ocu.rc_spd_w[Yr] * 0.06*dt;
			vmc_all.param.test_pos_flt[Yr] = LIMIT(vmc_all.param.test_pos_flt[Yr], 0.5*MIN_Y, 0.5*MAX_Y);

			vmc_all.param.test_pos_flt[Zr] = tar_air_z;
			vmc_all.param.test_pos_flt[Zr] = LIMIT(vmc_all.param.test_pos_flt[Zr], 0.9*MAX_Z, 1.1*MIN_Z);

			DigitalLPF(LIMIT(-vmc_all.att[ROLr] + vmc_all.tar_att_bias[ROLr], -66, 66), &att_test_use[ROLr], 10, dt);//
			DigitalLPF(LIMIT(-vmc_all.att[PITr] + vmc_all.tar_att_bias[PITr], -66, 66), &att_test_use[PITr], 10, dt);//
			//DigitalLPF(LIMIT(vmc_all.tar_spd.z*k_att_test[YAWr], -45, 45) * !en_yaw_test, &att_test_use[YAWr], 10, dt);

			float RTb_n_target[3][3];//
			RTb_n_target[0][0] = cosd(-att_test_use[PITr]);  RTb_n_target[0][1] = sind(-att_test_use[PITr])*sind(-att_test_use[ROLr]); RTb_n_target[0][2] = sind(-att_test_use[PITr])*cosd(-att_test_use[ROLr]);
			RTb_n_target[1][0] = 0;												   RTb_n_target[1][1] = cosd(-att_test_use[ROLr]);													 	RTb_n_target[1][2] = -sind(-att_test_use[ROLr]);
			RTb_n_target[2][0] = -sind(-att_test_use[PITr]); RTb_n_target[2][1] = cosd(-att_test_use[PITr])*sind(-att_test_use[ROLr]); RTb_n_target[2][2] = cosd(-att_test_use[PITr])*cosd(-att_test_use[ROLr]);

			END_POS angke_n[4];//			
			//for (i = 0; i < 4; i++)
			converV_b_to_n_RT(RTb_n_target, att_test_use[YAWr], vmc[i].flag_fb*vmc_all.H / 2, vmc[i].flag_rl*vmc_all.W / 2, 0, &angke_n[i].x, &angke_n[i].y, &angke_n[i].z);
			//---- 
			END_POS leg_2_angke_n[4];//
			//for (i = 0; i < 4; i++) {
			leg_2_angke_n[i].x = end_pos_n[i].x + vmc_all.param.test_pos_flt[Xr] - angke_n[i].x + vmc_all.param.param_vmc.side_off[Xr] * vmc[i].flag_fb * 0;
			leg_2_angke_n[i].y = end_pos_n[i].y + vmc_all.param.test_pos_flt[Yr] - angke_n[i].y + vmc_all.param.param_vmc.side_off[Yr] * vmc[i].flag_rl * 0;
			leg_2_angke_n[i].z = vmc_all.param.test_pos_flt[Zr] - angke_n[i].z;
			//}
			//----
			END_POS leg_2_angke_b[4];
			//for (i = 0; i < 4; i++) {
			converV_n_to_b_w_yaw(-att_test_use[YAWr], leg_2_angke_n[i].x, leg_2_angke_n[i].y, leg_2_angke_n[i].z,
				&leg_2_angke_b[i].x, &leg_2_angke_b[i].y, &leg_2_angke_b[i].z);
			leg_2_angke_b[i].x += vmc[i].flag_fb*vmc_all.H / 2;
			leg_2_angke_b[i].y += vmc[i].flag_rl*vmc_all.W / 2;
			//}
			//----
			END_POS leg_2_angke_leg[4];
			//for (i = 0; i < 4; i++)
			converV_b_to_leg(i, leg_2_angke_b[i].x, leg_2_angke_b[i].y, leg_2_angke_b[i].z,
				&leg_2_angke_leg[i].x, &leg_2_angke_leg[i].y, &leg_2_angke_leg[i].z);
#if !EN_END_SPD_MODE//我的VMC 直接输出期望角度到底层
			for (i = 0; i < 4; i++)
				inv_end_state_new(&vmc[i], leg_2_angke_leg[i].x, leg_2_angke_leg[i].y, leg_2_angke_leg[i].z,
					&vmc[i].tar_sita1, &vmc[i].tar_sita2, &vmc[i].tar_sita3);
#else
			//for (i = 0; i < 4; i++) {
			vmc[i].tar_epos_h = leg_2_angke_leg[i];//输出给阻抗控制器 进一步平滑滤波
			vmc[i].tar_epos_h.x = LIMIT(vmc[i].tar_epos_h.x+ tar_air_x*0, MIN_X, MAX_X);//期望设定位置限制幅度   
			vmc[i].tar_epos_h.z = LIMIT(vmc[i].tar_epos_h.z, MAX_Z, MIN_Z);
			//}
#endif
		}
	}

	for (i = 0; i < 10; i++)
		timer_task[i] += dt;
}