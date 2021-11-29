#include "gait_math.h"
#include "eso.h"
#include "locomotion_header.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

Gait_Pronk gait_pronk;

static END_POS end_pos_stand_h[4], end_pos_stand_n[4];
static char st_mode_reg = 0;

static float stand_gcheck_time[2] = { 0.01,0.05 };//TD  LF 着地时间判断与离地时间
static END_POS vmc_all_pos_n_air_st;
static END_POS vmc_all_spd_n_air_st;

void  Gait_Pronk_Active(void)
{
	int i = 0;
	vmc_all.gait_mode = PRONK;
	vmc_all.param.robot_mode = M_PRONK;
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

	gait_pronk.state = 0;
	gait_pronk.timer[0] = gait_pronk.timer[1] = gait_pronk.timer[2] = 0;
	gait_pronk.jump_height_off = 0.15;
	gait_pronk.jump_height = vmc_all.pos_n.z+ gait_pronk.jump_height_off;
	gait_pronk.jump_w = 0 / 57.3;
	gait_pronk.kp[Xr] = 100;
	gait_pronk.kp[Zr] = 150;
	gait_pronk.kp_T[Zr] = 10;
	gait_pronk.lf_time = 0.4;//s
	gait_pronk.couch_time = gait_pronk.lf_time*0.618;//s
	gait_pronk.jump_x_dis = 0.1*1;

	robotwb.cog_F_ff.x = 0;
	robotwb.cog_F_ff.z = 0;

	robotwb.cog_T_ff.z = 0;
#if RUN_WEBOTS
	printf("Pronk Gait Initing!!!!!!!\n");
#endif
}


void  Gait_Pronk_Update(float dt)
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
	//gait_pronk.jump_x_dis = ocu.rc_spd_w[Xr];

	//空中模式下的状态估计
	vmc_all_pos_n_air_st.x = (vmc[0].epos_n.x + vmc[1].epos_n.x + vmc[2].epos_n.x + vmc[3].epos_n.x) / 4;
	vmc_all_pos_n_air_st.z = -(vmc[0].epos_n.z + vmc[1].epos_n.z + vmc[2].epos_n.z + vmc[3].epos_n.z) / 4;

	vmc_all_spd_n_air_st.x = (vmc[0].spd_n.x + vmc[1].spd_n.x + vmc[2].spd_n.x + vmc[3].spd_n.x) / 4;
	vmc_all_spd_n_air_st.z = (vmc[0].spd_n.z + vmc[1].spd_n.z + vmc[2].spd_n.z + vmc[3].spd_n.z) / 4;

	st_mode_reg = vmc_all.gait_mode;

	static float en_force_timer[2] = { 0 };
#if STAND_GROUND_CHECK_TEST//力控标志位赋值
	if ((vmc_all.ground_num >=2
		|| (vmc[0].ground&&vmc[3].ground)
		|| (vmc[1].ground&&vmc[2].ground)
		)&& !stand_force_enable_flag[4])//着地
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
		tar_air_z=vmc_all.param.test_pos_flt[Zr] = -fabs(vmc_all_pos_n_air_st.z);

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
		tar_air_z=vmc_all.param.test_pos_flt[Zr] = -fabs(vmc_all_pos_n_air_st.z);//足端位置

		stand_force_enable_flag[4] = 0;
#if RUN_WEBOTS
		printf(" Air Mode!!\n");
#endif
	}
#endif	

	gait_pronk.jump_trig=1;
	switch (gait_pronk.state)
	{
	case 0:
		if (gait_pronk.jump_trig&&fabs(vmc_all.att[ROLr]) < 5) {
			gait_pronk.jump_trig = 0;
			gait_pronk.state++;
			gait_pronk.timer[0] = gait_pronk.timer[1] = gait_pronk.timer[2] = 0;
			#if RUN_WEBOTS
				printf("Pronk Planning...\n");
			#endif
		}
		break;
	case 1:
		gait_pronk.timer[0] += dt;


		if(gait_pronk.timer[0]>0.15){
			gait_pronk.timer[0] = 0;
			gait_pronk.state++;

			gait_pronk.st_height= vmc_all.pos_n.z;
			gait_pronk.couch_height = fabs(MIN_Z)*1.3;
#if RUN_WEBOTS
			printf("Pronk Start & Couch!!\n");
#endif
		}

	break;
	case 2:
		gait_pronk.timer[0] += dt;
		sincos_temp[0] =limitw (gait_pronk.timer[0] / gait_pronk.couch_time,0,1) * 90;
		vmc_all.tar_pos.x = -gait_pronk.jump_x_dis/4*sind(sincos_temp[0])*0;
		vmc_all.tar_pos.z = gait_pronk.st_height - (gait_pronk.st_height - gait_pronk.couch_height)*sind(sincos_temp[0]);
		if (gait_pronk.timer[0] >= gait_pronk.couch_time) {
			gait_pronk.timer[0] = 0;
			gait_pronk.state++;
			gait_pronk.ch_height = vmc_all.pos_n.z;
			gait_pronk.jump_power =(gait_pronk.jump_height - gait_pronk.couch_height)* Mw * gw;

			gait_pronk.jump_avg_F[Xr] = gait_pronk.kp[Xr] *gait_pronk.jump_x_dis* Mw * gw;
			gait_pronk.jump_avg_F[Zr] = gait_pronk.kp[Zr] *gait_pronk.jump_power;
			gait_pronk.jump_avg_T[Zr] = gait_pronk.kp_T[Zr] *gait_pronk.jump_w;
#if RUN_WEBOTS
			printf(" Couch Finish & Start to Jump!!\n");
#endif
		}

	break;

	case 3:
		gait_pronk.timer[0] += dt*2;
		gait_pronk.timer[1] += dt;
		sincos_temp[0] = limitw(gait_pronk.timer[0] / gait_pronk.lf_time, 0, 1) * 90;
		sincos_temp[1] = limitw(gait_pronk.timer[1] / gait_pronk.lf_time, 0, 1) * 90;
		vmc_all.tar_pos.x = gait_pronk.jump_x_dis/4* sind(sincos_temp[0])*0;
		vmc_all.tar_pos.z = gait_pronk.ch_height + (gait_pronk.jump_height - gait_pronk.ch_height)*sind(sincos_temp[0]);//
		//vmc_robot_p.pos_z.kp = 350;//Z高度刚度
		//vmc_robot_p.pos_z.ki = 5;
		//vmc_robot_p.pos_z.kd = 35;
		//pos_force_p.q_pid_st_stance.kp = 0.01;
		//pos_force_p.q_pid_st_stance.kd = 0;
		robotwb.cog_F_ff.x = gait_pronk.jump_avg_F[Xr] * sind(sincos_temp[1]);
		robotwb.cog_F_ff.z = gait_pronk.jump_avg_F[Zr] * sind(sincos_temp[1]);

		robotwb.cog_T_ff.z = gait_pronk.jump_avg_T[Zr] * sind(sincos_temp[1]);
		//printf("fz=%f\n", robotwb.cog_F_ff.z);
		if (gait_pronk.timer[0] >= gait_pronk.lf_time*2
			|| (vmc_all.ground_num<=1&& gait_pronk.timer[0] >= gait_pronk.lf_time*0.618)
			|| (vmc_all.pos_n.z>=gait_pronk.jump_height)) {
			gait_pronk.timer[1]=gait_pronk.timer[0] = 0;
			gait_pronk.state++;
			stand_force_enable_flag[4] = 0;
			tar_air_z = -fabs(MIN_Z)*1.3;

			vmc_all.tar_pos.x = 0;
			vmc_all.tar_pos.z = -tar_air_z;

			gait_pronk.lf_spd[Xr] = vmc_all.spd_n.x;
			gait_pronk.lf_spd[Zr] = vmc_all.spd_n.z;
			gait_pronk.fly_time = 2 * fabs(vmc_all.spd_n.z) / gw;
			gait_pronk.fly_time_real = 0;
			tar_air_x = gait_pronk.lf_spd[Xr]* gait_pronk.fly_time/2;

#if RUN_WEBOTS
			printf("fly_time=%f lf_spdx=%f lf_spdz=%f\n", gait_pronk.fly_time, gait_pronk.lf_spd[Xr], gait_pronk.lf_spd[Zr]);
			printf(" Jump Finish!!\n");
#endif
			robotwb.cog_F_ff.x = 0;
			robotwb.cog_F_ff.z = 0;

			robotwb.cog_T_ff.z = 0;
		}
	break;
	case 4:

		vmc_all.pos_n.x = vmc_all_pos_n_air_st.x;
		vmc_all.pos_n.z = vmc_all_pos_n_air_st.z;
		vmc_all.spd_n.x = 0;
		vmc_all.spd_n.z = 0;

		if(vmc_all.ground_num<=2)
			gait_pronk.fly_time_real += dt;

		if (stand_force_enable_flag[4]&& vmc_all.ground_num>=4&&fabs(vmc_all.att[ROLr])<5) {
			vmc_all.tar_pos.x = tar_air_x= 0;
			vmc_all.tar_pos.z = 0.7*fabs(MAX_Z);
			gait_pronk.state++;

#if RUN_WEBOTS
			printf(" Ground && Reheight fly_time=%f!!\n", gait_pronk.fly_time_real);
#endif
		}
	break;
	case 5:
		gait_pronk.timer[0] += dt;
		if (gait_pronk.timer[0] > 0.5
			&&fabs(vmc_all.att[ROLr]) < 5
			&&fabs(fabs(vmc_all.tar_pos.z)- vmc_all.pos_n.z)<0.01) {
			gait_pronk.timer[0] = 0;
			gait_pronk.state=0;
		}
	break;


	default:
		break;
	}



	if (stand_force_enable_flag[4] == 0)//空中位置模式式
	{
		for (i = 0; i < 4; i++) {//期望末端位置
			end_pos_n[i].x = end_pos_stand_n[i].x + vmc_all.param.param_vmc.posz_idle[Xr];
			end_pos_n[i].y = end_pos_stand_n[i].y + vmc_all.param.param_vmc.posz_idle[Yr];
			end_pos_n[i].z = end_pos_stand_n[i].z;
		}

		DigitalLPF(-ocu.rc_att_w[PITr] * vmc_all.param.MAX_PIT, &vmc_all.param.test_att_flt[PITr], 0.85, dt);
		DigitalLPF(-ocu.rc_att_w[ROLr] * vmc_all.param.MAX_ROL, &vmc_all.param.test_att_flt[ROLr], 0.85, dt);
		DigitalLPF(ocu.rate_yaw_w*vmc_all.param.MAX_YAW, &vmc_all.param.test_att_flt[YAWr], 0.85, dt);

		//位置
		vmc_all.param.test_pos_flt[Xr] = tar_air_x;
		vmc_all.param.test_pos_flt[Xr] = LIMIT(vmc_all.param.test_pos_flt[Xr], 0.35*MIN_X, 0.35*MAX_X);

		vmc_all.param.test_pos_flt[Yr] += ocu.rc_spd_w[Yr] * 0.06*dt;
		vmc_all.param.test_pos_flt[Yr] = LIMIT(vmc_all.param.test_pos_flt[Yr], 0.5*MIN_Y, 0.5*MAX_Y);

		vmc_all.param.test_pos_flt[Zr] = tar_air_z;
		vmc_all.param.test_pos_flt[Zr] = LIMIT(vmc_all.param.test_pos_flt[Zr], 0.9*MAX_Z, 1.1*MIN_Z);

		DigitalLPF(LIMIT(vmc_all.att[ROLr] + vmc_all.tar_att_bias[ROLr], -66, 66), &att_test_use[ROLr], 10, dt);//
		DigitalLPF(LIMIT(vmc_all.att[PITr] + vmc_all.tar_att_bias[PITr], -66, 66), &att_test_use[PITr], 10, dt);//
		//DigitalLPF(LIMIT(vmc_all.tar_spd.z*k_att_test[YAWr], -45, 45) * !en_yaw_test, &att_test_use[YAWr], 10, dt);

		float RTb_n_target[3][3];//
		RTb_n_target[0][0] = cosd(-att_test_use[PITr]);  RTb_n_target[0][1] = sind(-att_test_use[PITr])*sind(-att_test_use[ROLr]); RTb_n_target[0][2] = sind(-att_test_use[PITr])*cosd(-att_test_use[ROLr]);
		RTb_n_target[1][0] = 0;												   RTb_n_target[1][1] = cosd(-att_test_use[ROLr]);													 	RTb_n_target[1][2] = -sind(-att_test_use[ROLr]);
		RTb_n_target[2][0] = -sind(-att_test_use[PITr]); RTb_n_target[2][1] = cosd(-att_test_use[PITr])*sind(-att_test_use[ROLr]); RTb_n_target[2][2] = cosd(-att_test_use[PITr])*cosd(-att_test_use[ROLr]);

		END_POS angke_n[4];//			
		for (i = 0; i < 4; i++)
			converV_b_to_n_RT(RTb_n_target, att_test_use[YAWr], vmc[i].flag_fb*vmc_all.H / 2, vmc[i].flag_rl*vmc_all.W / 2, 0, &angke_n[i].x, &angke_n[i].y, &angke_n[i].z);
		//---- 
		END_POS leg_2_angke_n[4];//
		for (i = 0; i < 4; i++) {
			leg_2_angke_n[i].x = end_pos_n[i].x + vmc_all.param.test_pos_flt[Xr] - angke_n[i].x + vmc_all.param.param_vmc.side_off[Xr] * vmc[i].flag_fb * 0;
			leg_2_angke_n[i].y = end_pos_n[i].y + vmc_all.param.test_pos_flt[Yr] - angke_n[i].y + vmc_all.param.param_vmc.side_off[Yr] * vmc[i].flag_rl * 0;
			leg_2_angke_n[i].z = vmc_all.param.test_pos_flt[Zr] - angke_n[i].z;
		}
		//----
		END_POS leg_2_angke_b[4];
		for (i = 0; i < 4; i++) {
			converV_n_to_b_w_yaw(-att_test_use[YAWr], leg_2_angke_n[i].x, leg_2_angke_n[i].y, leg_2_angke_n[i].z,
				&leg_2_angke_b[i].x, &leg_2_angke_b[i].y, &leg_2_angke_b[i].z);
			leg_2_angke_b[i].x += vmc[i].flag_fb*vmc_all.H / 2;
			leg_2_angke_b[i].y += vmc[i].flag_rl*vmc_all.W / 2;
		}
		//----
		END_POS leg_2_angke_leg[4];
		for (i = 0; i < 4; i++)
			converV_b_to_leg(i, leg_2_angke_b[i].x, leg_2_angke_b[i].y, leg_2_angke_b[i].z,
				&leg_2_angke_leg[i].x, &leg_2_angke_leg[i].y, &leg_2_angke_leg[i].z);
#if !EN_END_SPD_MODE//我的VMC 直接输出期望角度到底层
		for (i = 0; i < 4; i++)
			inv_end_state_new(&vmc[i], leg_2_angke_leg[i].x, leg_2_angke_leg[i].y, leg_2_angke_leg[i].z,
				&vmc[i].tar_sita1, &vmc[i].tar_sita2, &vmc[i].tar_sita3);
#else
		for (i = 0; i < 4; i++) {
			vmc[i].tar_epos_h = leg_2_angke_leg[i];//输出给阻抗控制器 进一步平滑滤波
			vmc[i].tar_epos_h.x = LIMIT(vmc[i].tar_epos_h.x, MIN_X, MAX_X);//期望设定位置限制幅度   
			vmc[i].tar_epos_h.z = LIMIT(vmc[i].tar_epos_h.z, MAX_Z, MIN_Z);
		}
#endif
	}

	for (i = 0; i < 10; i++)
		timer_task[i] += dt;
}