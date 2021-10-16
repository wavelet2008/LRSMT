#include "locomotion_header.h"
#include "gait_math.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define SINGLE_LEG_TEST 0//WS Ϊ0
float lf_spd = 0.3;
float td_spd = 0.3;
#if !EN_PLAN_USE_JERK
float lf_spd_trot = 0.8;
float td_spd_trot = 0.8;
#else
float lf_spd_trot = 0.48;
float td_spd_trot = 0.8;
#endif
float lf_spd_walk = 0.03;
float td_spd_walk = 0.03;

float lf_spd_back_x = 0.00;
float lf_offz = 0.15;
float td_offz = 0.15;

char EN_Y_SLIP = 1;//0->������SLIPģ��ǰ��
char EN_TD_XOFF = 1;
char EN_SLIP_CONTROL = 1;//�������ڿ���
char EN_LIFT_Y_CONTROL = 0;
char EN_TD_Y_CONTROL = 0;
float k_spd_err = 55;//57.3;//�ٶȷ���

float max_end_spd = 2;
float spd_end_test[3] = { 0,0,0 };
float sw_time_set = 8;
float EN_SPDN_TRIG[2] = { 1,1 };//{1,1}摆动跟随地面速度

float pd_y[2] = { 0.19,0.08 };//{0.09,0.03};

float k_y_roll = 0.00086;

float kp_rc = 0;
float kp_force = 0;
float yaw_trig_k = 0;//-0.025;//0.05;
float FLT_SW_END = 0;
float trig_z_flag = 99;//99;//0; 1 99   -1依据地形缩腿逆解  99柔顺
float td_time = 0;
float lf_time = 0;
float spd_kk[2] = { 1,1 };

float ground_att[2] = { 0,0 };

char leg_lock_n_trigt[4] = { 0,0,0,0 };
char leg_lock_n_trig_regt[4] = { 0,0,0,0 };
END_POS end_lock_nt[4];

char cnt_reset_pos[4] = { 0 };
float cnt_online_plan[4] = { 0 };

void cal_vir_leg_pos(VMC* in, END_POS *td_vir_n, END_POS *lf_vir_n)//����������λ��
{
}

float tar_spd[3];
void cal_tar_end_pos_n_v1_sel(char leg, char recal, float dt)//���������� ȫ��
{
	float att_off = 0;
	float cog_off = 0;
	float spd_off[2] = { 0 }, acc_off[2] = { 0 };
	float spd_off_all[2] = { 0 }, cog_off_all[2] = { 0 };
	float size_off[2] = { 0 };

	float cof_off_all_use;
	float yaw_off[2];
	float temp_h = 0;
	int i = 0, j = 0;
	END_POS tar_epos_slip_ncom;
	END_POS end_epos_gn[4];

	tar_spd[Xr] = vmc_all.param.tar_spd_use_rc.x;
	tar_spd[Yr] = vmc_all.param.tar_spd_use_rc.y;
	//----------------Raibert 
   //##################################################################################
	if ((tar_spd[Xr]) >= 0)
		spd_off_all[Xr] = vmc_all.param.cof_off_all[F];
	else
		spd_off_all[Xr] = vmc_all.param.cof_off_all[Bs];

	temp_h = fabs(vmc_all.pos_n.z);

	spd_off[Xr] = -my_sqrt((temp_h + vmc_all.ankle_pos_n[leg].z) / 9.81)*(tar_spd[Xr] - vmc_all.spd_n.x);//CP 反馈

	//printf("%f %f %f\n",tar_spd[Xr],vmc_all.spd_n.x,spd_off[Xr]);
	//spd_off[Xr]=0;
	att_off = LIMIT(tan((vmc_all.att_trig[PITr])*DEG_TO_RAD)*vmc_all.pos.z
		, MIN_X*0.8, MAX_X*0.8);//

	tar_epos_slip_ncom.x = LIMIT(
		tar_spd[Xr] *
		spd_kk[Xr] * (vmc_all.stance_time + vmc_all.delay_time[2] + vmc_all.gait_delay_time) / 2 +
		spd_off[Xr] +
		acc_off[Xr] +
		spd_off_all[Xr] +
		vmc_all.param.sw_com_off[Xr]
		, MIN_X, MAX_X);

	//########################################����###########################################
	if ((tar_spd[Yr]) >= 0)
		spd_off_all[Yr] = 0;
	else
		spd_off_all[Yr] = 0;

	temp_h = fabs(vmc_all.pos_n.z);
	spd_off[Yr] = vmc_all.kp_trig[0] * (tar_spd[Yr] - vmc_all.spd_n.y)*sqrt((temp_h) / 9.81);

	cog_off_all[Yr] = 0;//

	tar_epos_slip_ncom.y = LIMIT(
		vmc_all.spd_n.y*spd_kk[Yr] * (vmc_all.stance_time + vmc_all.delay_time[2] + vmc_all.gait_delay_time) / 2 +
		spd_off[Yr] +//
		acc_off[Yr] +
		spd_off_all[Yr] +
		cog_off_all[Yr]
		, MIN_Y, MAX_Y);
	//################################################################################
	tar_epos_slip_ncom.z = LIMIT(-fabs(vmc_all.pos_n.z), MAX_Z, MIN_Z);


	//################################################################################
	td_time = td_offz * vmc[i].delta_ht / td_spd;
	lf_time = lf_offz * vmc[i].delta_ht / lf_spd;

	float td_x_off = td_time * LIMIT(tar_spd[Xr], -MIN_X * 0.15, MAX_X*0.15);

	i = leg;
	vmc[i].tar_epos.x = tar_epos_slip_ncom.x;
	vmc[i].tar_epos.y = tar_epos_slip_ncom.y*EN_Y_SLIP;
	vmc[i].tar_epos.z = tar_epos_slip_ncom.z;
	//------------------------------------Xr-------------------------------------
	float leg_off = 0;
	if (vmc[i].flag_fb == 1)
		leg_off = vmc_all.param.param_vmc.leg_off[F];
	else
		leg_off = vmc_all.param.param_vmc.leg_off[Bs];

	if (vmc_all.param.stand_switch_flag[0] == 1) {
		if (vmc_all.gait_mode == WALK)
			size_off[Xr] = vmc[i].flag_fb*vmc_all.param.param_vmc.side_off_walk[Xr];
		else
			size_off[Xr] = vmc[i].flag_fb*vmc_all.param.param_vmc.side_off_stand[Xr];
	}
	else {
		size_off[Xr] = vmc[i].flag_fb*vmc_all.param.param_vmc.side_off[Xr];
	}
	vmc[i].tar_epos.x += size_off[Xr];

	yaw_off[Xr] = vmc_all.yaw_force*vmc[i].flag_rl*yaw_trig_k;
	vmc[i].tar_epos.x += yaw_off[Xr];

	vmc[i].tar_epos.x += td_x_off * EN_TD_XOFF;
	vmc[i].tar_epos.x += leg_off;
	//------------------------------------Yr--------------------------------------
	if (vmc_all.param.stand_switch_flag[0] == 1) {
		if (vmc_all.gait_mode == WALK)
			size_off[Yr] = vmc[i].flag_rl*vmc_all.param.param_vmc.side_off_walk[Yr];//
		else
			size_off[Yr] = vmc[i].flag_rl*vmc_all.param.param_vmc.side_off_stand[Yr];
	}
	else {
		size_off[Yr] = vmc[i].flag_rl*vmc_all.param.param_vmc.side_off[Yr];//
	}
	vmc[i].tar_epos.y += size_off[Yr];

	yaw_off[Yr] = vmc_all.yaw_force*vmc[i].flag_fb*yaw_trig_k;//
	vmc[i].tar_epos.y += yaw_off[Yr];
	//------------------------------------Zr--------------------------------------

//###############################################################################	
	END_POS ankle_pos_n[4];
	END_POS ankle_pos_b[4];
	END_POS end_epos_n[4];
	END_POS end_epos_b[4];
	END_POS end_epos_n1[4];
	END_POS end_epos_h[4];
	i = leg;
	//-----------------------------------------
	//
	ankle_pos_b[i].x = vmc[i].flag_fb*vmc_all.H / 2;
	ankle_pos_b[i].y = vmc[i].flag_rl*vmc_all.W / 2;
	ankle_pos_b[i].z = 0;
	//
	float RTb_n_target[3][3];//
	float att_rt_use[3];
#if 1//WS	
	ground_att[PITr] = vmc_all.ground_att_cmd[PITr] * 1;//落足地形转换
	ground_att[ROLr] = vmc_all.ground_att_cmd[ROLr] * 1;
	att_rt_use[PITr] = vmc_all.att_trig[PITr];// - 1 * ground_att[PITr];//(vmc_all.tar_att[PITr]+vmc_all.tar_att_bias[PITr]*1+vmc_all.tar_att_off[PITr]);
	att_rt_use[ROLr] = vmc_all.att_trig[ROLr];// - 1 * ground_att[ROLr];//(vmc_all.tar_att[ROLr]+vmc_all.tar_att_bias[ROLr]*1+vmc_all.tar_att_off[ROLr]);
	att_rt_use[YAWr] = 0;

	RTb_n_target[0][0] = cosd(-att_rt_use[PITr])*cosd(-att_rt_use[YAWr]);
	RTb_n_target[1][0] = -cosd(-att_rt_use[ROLr])*sind(-att_rt_use[YAWr]) + sind(-att_rt_use[PITr])*sind(-att_rt_use[ROLr])*cosd(-att_rt_use[YAWr]);
	RTb_n_target[2][0] = sind(-att_rt_use[ROLr])*sind(-att_rt_use[YAWr]) + cosd(-att_rt_use[ROLr])*sind(-att_rt_use[PITr])*cosd(-att_rt_use[YAWr]);

	RTb_n_target[0][1] = cosd(-att_rt_use[PITr])*sind(-att_rt_use[YAWr]);
	RTb_n_target[1][1] = cosd(-att_rt_use[ROLr])*cosd(-att_rt_use[YAWr]) + sind(-att_rt_use[ROLr])*sind(-att_rt_use[PITr])*sind(-att_rt_use[YAWr]);
	RTb_n_target[2][1] = -sind(-att_rt_use[ROLr])*cosd(-att_rt_use[YAWr]) + cosd(-att_rt_use[ROLr])*sind(-att_rt_use[PITr])*sind(-att_rt_use[YAWr]);

	RTb_n_target[0][2] = -sind(-att_rt_use[PITr]);
	RTb_n_target[1][2] = sind(-att_rt_use[ROLr])*cosd(-att_rt_use[PITr]);
	RTb_n_target[2][2] = cosd(-att_rt_use[ROLr])*cosd(-att_rt_use[PITr]);

	converV_b_to_n_RT(RTb_n_target, 0, ankle_pos_b[i].x, ankle_pos_b[i].y, ankle_pos_b[i].z,
		&ankle_pos_n[i].x, &ankle_pos_n[i].y, &ankle_pos_n[i].z);
#else
	ankle_pos_n = ankle_pos_b;
#endif

	end_epos_n[i].x = vmc[i].tar_epos.x + ankle_pos_n[i].x;
	end_epos_n[i].y = vmc[i].tar_epos.y + ankle_pos_n[i].y;
	end_epos_n[i].z = vmc[i].tar_epos.z + ankle_pos_n[i].z*trig_z_flag;
	if (trig_z_flag == 99)
		end_epos_n[i].z = vmc[i].epos_lf_n.z;//Ĭ�ϵ���ˮƽ����	

	if (ocu.key_y && !ocu.key_y_reg)
		leg_lock_n_trigt[0] = !leg_lock_n_trigt[0];
	leg_lock_n_trigt[0] = leg_lock_n_trigt[2] = ocu.key_y;//doghome
	//printf("leg_lock_n_trigt[0]=%d\n",leg_lock_n_trigt[0]);
	if (vmc_all.gait_mode == TROT || vmc_all.gait_mode == F_TROT) {
		leg_lock_n_trigt[2] = leg_lock_n_trigt[0];
		leg_lock_n_trigt[3] = leg_lock_n_trigt[1];
	}

	if (leg_lock_n_trigt[i] && !leg_lock_n_trig_regt[i])
		end_lock_nt[i] = vmc[i].epos_nn;

	if (leg_lock_n_trigt[i]) {//ȫ������̶�����
		end_epos_gn[i] = end_lock_nt[i];
	}
	leg_lock_n_trig_regt[i] = leg_lock_n_trigt[i];

	end_epos_n1[i].x = end_epos_gn[i].x - vmc_all.cog_pos_n.x;
	end_epos_n1[i].y = end_epos_gn[i].y - vmc_all.cog_pos_n.y;
	end_epos_n1[i].z = end_epos_gn[i].z;

	if (leg_lock_n_trigt[i]) {
		end_epos_n[i].x = end_epos_n1[i].x;
		end_epos_n[i].y = end_epos_n1[i].y;
	}

	vmc[i].tar_epos_n.x = end_epos_n[i].x;
	vmc[i].tar_epos_n.y = end_epos_n[i].y;
	if (!recal) {
		vmc[i].tar_epos_n.z = end_epos_n[i].z;
		//vmc[i].tar_epos_n.z += td_offz;
	}
	//printf("%f\n",vmc[i].tar_epos_n.x);	
}


float end_pos_y_correction(VMC* in, float ep_slip_n, float dt)
{
	return 0;
}

void trig_curve_global(VMC *in, float *x, float *y, float *z, float *vx, float *vy, float *vz, float dt)
{
	END_POS epos, epos_nn;
	float alfa = 0;
	float err[3], att_use[3];
	int id = in->param.id, temp = 0;;
	att_use[PITr] = vmc_all.att_vm[PITr];
	att_use[ROLr] = vmc_all.att_ctrl[ROLr];
	err[PITr] = LIMIT(vmc_all.tar_att[PITr] + vmc_all.tar_att_bias[PITr] + vmc_all.tar_att_off[PITr] - att_use[PITr], -24, 24);
	err[ROLr] = LIMIT(vmc_all.tar_att[ROLr] + vmc_all.tar_att_bias[ROLr] + vmc_all.tar_att_off[ROLr] - att_use[ROLr], -24, 24);
	err[2] = MAX(ABS(err[PITr]), ABS(err[ROLr]));

	in->param.time_trig += dt;
	if (in->param.time_trig < vmc_all.gait_time[1] - vmc_all.stance_time - dt)
	{
#if EN_PLAN_USE_JERK==0
		epos = cal_pos_tar_from_curve(in, vmc_all.gait_time[1] - vmc_all.stance_time, dt);
		*x = epos.x;
		*y = epos.y;
		*z = LIMIT(epos.z, MAX_Z, MIN_Z);
#else
		get_swing_jerk_5point(in, in->param.time_trig);
		*x = in->param.end_planner.pos_now[Xr];
		*y = in->param.end_planner.pos_now[Yr];
		*z = LIMIT(in->param.end_planner.pos_now[Zr], MAX_Z, MIN_Z);
#endif
	}
}

void espd_to_qspd(VMC *in, END_POS spd_end, float* spd_q, float dt)
{
	float temp[3] = { 0,0,0 };
	float spd_cmd[3];
	spd_cmd[Xr] = spd_end.x;
	spd_cmd[Yr] = spd_end.y;
	spd_cmd[Zr] = spd_end.z;

	switch (vmc_all.param.leg_dof) {
	case 2:
		spd_cmd[Yr] = 0;
		temp[D_LEG] = (-in->ijacobi22[0] * spd_cmd[Xr] * in->param.invert_knee_epos[Xr] + (-in->ijacobi22[1] * spd_cmd[Zr]));
		temp[X_LEG] = (-in->ijacobi22[2] * spd_cmd[Xr] * in->param.invert_knee_epos[Xr] + (-in->ijacobi22[3] * spd_cmd[Zr]));

		break;
	case 3:
		temp[D_LEG] = in->ijacobi33[0] * spd_cmd[Xr] * in->param.invert_knee_epos[Xr] +
			in->ijacobi33[1] * spd_cmd[Yr] * in->param.invert_knee_epos[Yr] +
			in->ijacobi33[2] * spd_cmd[Zr];
		temp[X_LEG] = in->ijacobi33[3] * spd_cmd[Xr] * in->param.invert_knee_epos[Xr] +
			in->ijacobi33[4] * spd_cmd[Yr] * in->param.invert_knee_epos[Yr] +
			in->ijacobi33[5] * spd_cmd[Zr];
		temp[T_LEG] = in->ijacobi33[6] * spd_cmd[Xr] * in->param.invert_knee_epos[Xr] +
			in->ijacobi33[7] * spd_cmd[Yr] * in->param.invert_knee_epos[Yr] +
			in->ijacobi33[8] * spd_cmd[Zr];
		break;
	}

#if 0 
	spd_q[D_LEG] = temp[D_LEG] * in->param.flt_toqrue + (1 - in->param.flt_toqrue)*spd_q[D_LEG];
	spd_q[X_LEG] = temp[X_LEG] * in->param.flt_toqrue + (1 - in->param.flt_toqrue)*spd_q[X_LEG];
	spd_q[T_LEG] = temp[T_LEG] * in->param.flt_toqrue + (1 - in->param.flt_toqrue)*spd_q[T_LEG];
#else
	DigitalLPF(temp[D_LEG], &spd_q[0], FLT_SW_END, dt);
	DigitalLPF(temp[X_LEG], &spd_q[1], FLT_SW_END, dt);
	DigitalLPF(temp[T_LEG], &spd_q[2], FLT_SW_END, dt);
#endif
}


void reset_sw_torque_as_now(VMC *in, float torque_now[3])
{
	in->param.spd_dj[D_LEG] = torque_now[D_LEG];
	in->param.spd_dj[X_LEG] = torque_now[X_LEG];
	in->param.spd_dj[T_LEG] = torque_now[T_LEG];
}


void trig_plan(char leg_sel_trig, float dt) {

	vmc[leg_sel_trig].epos_lf_n = vmc[leg_sel_trig].epos_n;
	vmc[leg_sel_trig].epos_lf_hn = vmc[leg_sel_trig].epos;
	cal_tar_end_pos_n_v1_sel(leg_sel_trig, 0, dt);
	vmc[leg_sel_trig].cnt_ss = 0;
	if (vmc_all.gait_mode == WALK)
	{
		lf_spd = lf_spd_walk;
		td_spd = td_spd_walk;
		EN_LIFT_Y_CONTROL = 1;
		EN_TD_Y_CONTROL = 1;
	}
	else
	{
		lf_spd = lf_spd_trot;
		td_spd = td_spd_trot;
		EN_LIFT_Y_CONTROL = 1;
		EN_TD_Y_CONTROL = 1;
	}
}

char trig_lift(char leg_sel_trig, float dt) {
	vmc[leg_sel_trig].ground = 0;
	stand_force_enable_flag[leg_sel_trig] = 0;

	vmc[leg_sel_trig].epos_spdd_n.x = LIMIT(-vmc_all.spd_n.x, -max_end_spd, max_end_spd)*EN_SPDN_TRIG[Xr];
	vmc[leg_sel_trig].epos_spdd_n.z = lf_spd;
	if (EN_SLIP_CONTROL&&EN_LIFT_Y_CONTROL)
		vmc[leg_sel_trig].epos_spdd_n.y = end_pos_y_correction(&vmc[leg_sel_trig], vmc[leg_sel_trig].epos_lf_n.y, dt);
	else
		vmc[leg_sel_trig].epos_spdd_n.y = LIMIT(vmc_all.spd_n.y, -max_end_spd, max_end_spd)*EN_SPDN_TRIG[Yr];

	if (vmc_all.param.leg_dof == 2)
		vmc[leg_sel_trig].epos_spdd_n.y = 0;
	vmc[leg_sel_trig].cnt_ss += dt;
	char diag_leg_id = 0;//对角判断
	/*2    0
	  3    1*/
	switch (leg_sel_trig)
	{
	case 0:
		diag_leg_id = 3;
		break;
	case 1:
		diag_leg_id = 2;
		break;
	case 2:
		diag_leg_id = 1;
		break;
	case 3:
		diag_leg_id = 0;
		break;
	}

	if (fabs(vmc[leg_sel_trig].epos_n.z - vmc[leg_sel_trig].epos_lf_n.z) >= lf_offz * vmc[leg_sel_trig].delta_ht
		|| vmc[diag_leg_id].param.trig_state == 2
		|| vmc[leg_sel_trig].cnt_ss >= lf_time
		) {
		vmc[leg_sel_trig].st_pos = vmc[leg_sel_trig].epos_sw_st_n = vmc[leg_sel_trig].epos_n;//起点
		vmc[leg_sel_trig].tar_pos = vmc[leg_sel_trig].tar_epos_n;
		vmc[leg_sel_trig].param.time_trig = dt;
		vmc[leg_sel_trig].param.ground_state = vmc[leg_sel_trig].param.ground_state = 0;
	
		vmc[leg_sel_trig].param.tar_epos_n_reg = vmc[leg_sel_trig].param.tar_epos_n;
#if EN_PLAN_USE_JERK
		swing_jerk_planner_5point(&vmc[leg_sel_trig], lf_spd, td_spd, vmc_all.gait_time[1] - vmc_all.stance_time);
#else
		cal_curve_from_pos_new(&vmc[leg_sel_trig], vmc[leg_sel_trig].tar_pos, vmc_all.gait_time[1] - vmc_all.stance_time - lf_time);
#endif	
		vmc[leg_sel_trig].cnt_ss = dt;
		vmc[leg_sel_trig].param.tar_epos_n_reg = vmc[leg_sel_trig].param.tar_epos_n;
		cnt_online_plan[leg_sel_trig] = 0;
		return 1;
	}
	else
		return 0;
}


char trig_swing(char leg_sel_trig, float dt) {
	END_POS end_spd_n;
	end_spd_n.x = end_spd_n.y = end_spd_n.z = 0;

#if EN_ONLINE_SW_PLAN
	cnt_online_plan[leg_sel_trig] += dt;
	if (cnt_online_plan[leg_sel_trig] >= RE_PLAN_DT) {
		trig_plan_online(leg_sel_trig, cnt_online_plan[leg_sel_trig]);
		cnt_online_plan[leg_sel_trig] = 0;
	}
#endif

	trig_curve_global(&vmc[leg_sel_trig],//计算一次来获取轨迹初始值
		&vmc[leg_sel_trig].param.tar_epos_n.x,
		&vmc[leg_sel_trig].param.tar_epos_n.y,
		&vmc[leg_sel_trig].param.tar_epos_n.z,
		&vmc[leg_sel_trig].epos_spdd_n.x,
		&vmc[leg_sel_trig].epos_spdd_n.y,
		&vmc[leg_sel_trig].epos_spdd_n.z,
		dt);

#if SW_WITH_REAL_FB//用实际反馈规划足端轨迹容易发散 位置闭环 {N}系
	vmc[leg_sel_trig].epos_spdd_n.x = end_spd_n.x = (vmc[leg_sel_trig].param.tar_epos_n.x - vmc[leg_sel_trig].epos_n.x) / dt;
	end_spd_n.y = (vmc[leg_sel_trig].param.tar_epos_n.y - vmc[leg_sel_trig].epos_n.y) / dt;//δʹ��
	vmc[leg_sel_trig].epos_spdd_n.z = end_spd_n.z = (vmc[leg_sel_trig].param.tar_epos_n.z - vmc[leg_sel_trig].epos_n.z) / dt;
#else//微分轨迹规划速度
	vmc[leg_sel_trig].epos_spdd_n.x = end_spd_n.x = (vmc[leg_sel_trig].param.tar_epos_n.x - vmc[leg_sel_trig].param.tar_epos_n_reg.x) / dt;
	vmc[leg_sel_trig].epos_spdd_n.z = end_spd_n.z = (vmc[leg_sel_trig].param.tar_epos_n.z - vmc[leg_sel_trig].param.tar_epos_n_reg.z) / dt;

	vmc[leg_sel_trig].param.tar_epos_n_reg = vmc[leg_sel_trig].param.tar_epos_n;
#endif

	if (EN_SLIP_CONTROL)
		vmc[leg_sel_trig].epos_spdd_n.y = end_pos_y_correction(&vmc[leg_sel_trig], vmc[leg_sel_trig].param.tar_epos_n.y, dt);
	else
		vmc[leg_sel_trig].epos_spdd_n.y = end_spd_n.y;

	if (vmc_all.param.leg_dof == 2)
		vmc[leg_sel_trig].epos_spdd_n.y = 0;

	if (vmc[leg_sel_trig].param.time_trig >= vmc_all.gait_time[1] - vmc_all.stance_time)//摆动结束 切换TD
	{
		vmc[leg_sel_trig].param.time_trig = dt;
		vmc[leg_sel_trig].cnt_ss = dt;
		vmc[leg_sel_trig].epos_sw_end_n = vmc[leg_sel_trig].epos_n;
		return 1;
	}

	if (vmc[leg_sel_trig].param.time_trig >= (vmc_all.gait_time[1] - vmc_all.stance_time)*SW_LOW_RATE
		&& (vmc_all.use_ground_sensor_new == 1 && vmc[leg_sel_trig].is_touch)) {
		vmc[leg_sel_trig].cnt_ss = 0;
		vmc[leg_sel_trig].epos_spdd_n.x = vmc[leg_sel_trig].epos_spdd_n.y = vmc[leg_sel_trig].epos_spdd_n.z = 0;
		vmc[leg_sel_trig].epos_td_n = vmc[leg_sel_trig].epos_n;
		vmc[leg_sel_trig].epos_td_hn = vmc[leg_sel_trig].epos;
		vmc[leg_sel_trig].param.time_trig = 0;
#if !GROUND_AFTER_TRIG			
		vmc[leg_sel_trig].ground = 1;
		stand_force_enable_flag[leg_sel_trig] = 1;
		reset_tar_pos(leg_sel_trig);
#endif
		return 2;
	}
	return 0;
}

char trig_td(char leg_sel_trig, float dt) {
	vmc[leg_sel_trig].cnt_ss += dt;

	vmc[leg_sel_trig].epos_spdd_n.x = LIMIT(-vmc_all.spd_n.x, -max_end_spd, max_end_spd)*EN_SPDN_TRIG[Xr];
	vmc[leg_sel_trig].epos_spdd_n.z = -td_spd;

	if (EN_SLIP_CONTROL&&EN_TD_Y_CONTROL)
		vmc[leg_sel_trig].epos_spdd_n.y = end_pos_y_correction(&vmc[leg_sel_trig], vmc[leg_sel_trig].epos_sw_end_n.y, dt);
	else
		vmc[leg_sel_trig].epos_spdd_n.y = LIMIT(vmc_all.spd_n.y, -max_end_spd, max_end_spd)*EN_SPDN_TRIG[Yr];

	if (vmc_all.param.leg_dof == 2)
		vmc[leg_sel_trig].epos_spdd_n.y = 0;
	//printf("TD: %f %f\n",vmc[leg_sel_trig].epos_spdd_n.x,vmc[leg_sel_trig].epos_n.x);			
	if (fabs(vmc[leg_sel_trig].epos_n.z) >= fabs(vmc[leg_sel_trig].epos_sw_end_n.z) + td_offz  * vmc[leg_sel_trig].delta_ht) {
		if (vmc_all.use_ground_sensor_new == 0) {
			vmc[leg_sel_trig].cnt_ss = 0;
			vmc[leg_sel_trig].epos_spdd_n.x = vmc[leg_sel_trig].epos_spdd_n.y = vmc[leg_sel_trig].epos_spdd_n.z = 0;
			vmc[leg_sel_trig].epos_td_n = vmc[leg_sel_trig].epos_n;
			vmc[leg_sel_trig].epos_td_hn = vmc[leg_sel_trig].epos;
			vmc[leg_sel_trig].param.time_trig = 0;
#if !GROUND_AFTER_TRIG			
			vmc[leg_sel_trig].ground = 1; 
			stand_force_enable_flag[leg_sel_trig] = 1;
			reset_tar_pos(leg_sel_trig);
#endif

			return 1;//�л�֧��
		}//高度着地
	}

	if (vmc[leg_sel_trig].cnt_ss > vmc_all.stance_time*0.25&&vmc_all.use_ground_sensor_new == 0)//超时 不使用传感器
	{
		vmc[leg_sel_trig].cnt_ss = 0;
		vmc[leg_sel_trig].epos_spdd_n.x = vmc[leg_sel_trig].epos_spdd_n.y = vmc[leg_sel_trig].epos_spdd_n.z = 0;
		vmc[leg_sel_trig].epos_td_n = vmc[leg_sel_trig].epos_n; 
		vmc[leg_sel_trig].epos_td_hn = vmc[leg_sel_trig].epos;
		vmc[leg_sel_trig].param.time_trig = 0;
#if !GROUND_AFTER_TRIG			
		vmc[leg_sel_trig].ground = 1; 
		stand_force_enable_flag[leg_sel_trig] = 1;
		reset_tar_pos(leg_sel_trig);
#endif

		return 1;//�л�֧��
	}
	//-------------------------------------------------使用传感器----------------------------------------------------------
	if ((vmc_all.use_ground_sensor_new == 1 && vmc[leg_sel_trig].is_touch)) {//着地
		vmc[leg_sel_trig].cnt_ss = 0;
		vmc[leg_sel_trig].epos_spdd_n.x = vmc[leg_sel_trig].epos_spdd_n.y = vmc[leg_sel_trig].epos_spdd_n.z = 0;
		vmc[leg_sel_trig].epos_td_n = vmc[leg_sel_trig].epos_n; 
		vmc[leg_sel_trig].epos_td_hn = vmc[leg_sel_trig].epos;
		vmc[leg_sel_trig].param.time_trig = 0;
#if !GROUND_AFTER_TRIG			
		vmc[leg_sel_trig].ground = 1; 
		stand_force_enable_flag[leg_sel_trig] = 1;
		reset_tar_pos(leg_sel_trig);
#endif

		//printf("TD End Leg=%d\n", leg_sel_trig);
		return 1;//�л�֧��	
	}


	if (vmc[leg_sel_trig].cnt_ss > vmc_all.stance_time * 2 && vmc_all.use_ground_sensor_new == 1)//超时 
	{
		vmc[leg_sel_trig].cnt_ss = 0;
		vmc[leg_sel_trig].epos_spdd_n.x = vmc[leg_sel_trig].epos_spdd_n.y = vmc[leg_sel_trig].epos_spdd_n.z = 0;
		vmc[leg_sel_trig].epos_td_n = vmc[leg_sel_trig].epos_n; 
		vmc[leg_sel_trig].epos_td_hn = vmc[leg_sel_trig].epos;
		vmc[leg_sel_trig].param.time_trig = 0;
#if !GROUND_AFTER_TRIG			
		vmc[leg_sel_trig].ground = 1;
		stand_force_enable_flag[leg_sel_trig] = 1;
		reset_tar_pos(leg_sel_trig);
#endif

		return 1; 
	}

	char diag_leg_id = 0;//对角判断
	/*2    0
	  3    1*/
	switch (leg_sel_trig)
	{
	case 0:
		diag_leg_id = 3;
		break;
	case 1:
		diag_leg_id = 2;
		break;
	case 2:
		diag_leg_id = 1;
		break;
	case 3:
		diag_leg_id = 0;
		break;
	}

	if (
		0 &&
		(fabs(vmc[leg_sel_trig].epos.z) >= fabs(MAX_Z)*0.95)) {
		vmc[leg_sel_trig].epos_spdd_n.z = 0;

		if (vmc[diag_leg_id].is_touch) {
			vmc[leg_sel_trig].epos_spdd_n.x = 0;
			vmc[leg_sel_trig].epos_spdd_n.y = 0;
			vmc[leg_sel_trig].epos_td_n = vmc[leg_sel_trig].epos_n; 
			vmc[leg_sel_trig].cnt_ss = 0;
			vmc[leg_sel_trig].epos_td_hn = vmc[leg_sel_trig].epos;
			vmc[leg_sel_trig].param.time_trig = 0;
#if !GROUND_AFTER_TRIG			
			vmc[leg_sel_trig].ground = 1; 
			stand_force_enable_flag[leg_sel_trig] = 1;
			reset_tar_pos(leg_sel_trig);
#endif

			return 1;//
		}
	}
	return 0;
}

//---------------------------------------------------通用算法
void swing_spd_control(char leg_sel_trig, float dt)//速度输出
{
	vmc[leg_sel_trig].epos_spdd_n.x = -LIMIT(vmc[leg_sel_trig].epos_spdd_n.x, -max_end_spd, max_end_spd);
	vmc[leg_sel_trig].epos_spdd_n.y = -LIMIT(vmc[leg_sel_trig].epos_spdd_n.y, -max_end_spd, max_end_spd)*(vmc_all.param.leg_dof == 3);
	vmc[leg_sel_trig].epos_spdd_n.z = -LIMIT(vmc[leg_sel_trig].epos_spdd_n.z, -max_end_spd, max_end_spd);
#if EN_ROTATE_END_COMPASS_SW//旋转速度补偿
	Vect3 Hip_b;
	Vect3 com;
	com.x = com.y = com.z = 0;
	Hip_b.x = com.x - vmc[leg_sel_trig].epos_b.x;
	Hip_b.y = com.y - vmc[leg_sel_trig].epos_b.y;
	Hip_b.z = com.z - vmc[leg_sel_trig].epos_b.z;
	 
	Vect3 w_b;
	w_b.x = vmc_all.att_rate[ROLr] * 0.0173 * 1;
	w_b.y = vmc_all.att_rate[PITr] * 0.0173 * -1;
	w_b.z = vmc_all.att_rate[YAWr] * 0.0173 * 0;
	float w_cross_b[3][3] = { 0 };
	vect3_2_cross(w_b,w_cross_b);
	Vect3 hip_rotate_spd_b, hip_rotate_spd_n;

	matrx33_mult_vect3(w_cross_b, Hip_b, &hip_rotate_spd_b);

	converV_b_to_n(hip_rotate_spd_b.x, hip_rotate_spd_b.y, hip_rotate_spd_b.z,
		&hip_rotate_spd_n.x, &hip_rotate_spd_n.y, &hip_rotate_spd_n.z);//转换速度直接输出到IMP  控制频率不够目前？？	
	if (leg_sel_trig ==2&&0) {
		printf("x=%f  y=%f  z=%f\n", vmc[leg_sel_trig].epos_b.x, vmc[leg_sel_trig].epos_b.y, vmc[leg_sel_trig].epos_b.z);
		printf("bx=%f  by=%f  bz=%f\n", hip_rotate_spd_b.x, hip_rotate_spd_b.y, hip_rotate_spd_b.z);
		printf("nx=%f  ny=%f  nz=%f\n", hip_rotate_spd_n.x, hip_rotate_spd_n.y, hip_rotate_spd_n.z);
	}
	vmc[leg_sel_trig].epos_spdd_n.x -= hip_rotate_spd_n.x * 1;
	vmc[leg_sel_trig].epos_spdd_n.y -= hip_rotate_spd_n.y * 0;
	vmc[leg_sel_trig].epos_spdd_n.z -= hip_rotate_spd_n.z * 1;

#endif

#if 1
#if F_SWING_WITH_ROLL
	vmc[leg_sel_trig].epos_spdd_n.z /= cosd(LIMIT(vmc_all.att_trig[ROLr], -ROLL_LIMIT_COM, ROLL_LIMIT_COM));
#endif
	converV_n_to_b(vmc[leg_sel_trig].epos_spdd_n.x, vmc[leg_sel_trig].epos_spdd_n.y, vmc[leg_sel_trig].epos_spdd_n.z,
		&vmc[leg_sel_trig].epos_spdd_b.x, &vmc[leg_sel_trig].epos_spdd_b.y, &vmc[leg_sel_trig].epos_spdd_b.z);//转换速度直接输出到IMP  控制频率不够目前？？	
#else
	converV_n_to_b_noroll(vmc[leg_sel_trig].epos_spdd_n.x, vmc[leg_sel_trig].epos_spdd_n.y, vmc[leg_sel_trig].epos_spdd_n.z,
		&vmc[leg_sel_trig].epos_spdd_b.x, &vmc[leg_sel_trig].epos_spdd_b.y, &vmc[leg_sel_trig].epos_spdd_b.z);//转换速度直接输出到IMP  控制频率不够目前？？	
#endif
	vmc[leg_sel_trig].tar_epos_h.x += -vmc[leg_sel_trig].epos_spdd_b.x*dt;//积分摆动轨迹  用于作为IMP的期望轨迹！！！
	vmc[leg_sel_trig].tar_epos_h.z += -vmc[leg_sel_trig].epos_spdd_b.z*dt;

	vmc[leg_sel_trig].tar_epos_h.x = LIMIT(vmc[leg_sel_trig].tar_epos_h.x, MIN_X, MAX_X);//位置限制幅度  输出到IMP摆动控制
	vmc[leg_sel_trig].tar_epos_h.y = 0;
	vmc[leg_sel_trig].tar_epos_h.z = LIMIT(vmc[leg_sel_trig].tar_epos_h.z, MAX_Z, MIN_Z);//位置限制幅度

	espd_to_qspd(&vmc[leg_sel_trig], vmc[leg_sel_trig].epos_spdd_b, vmc[leg_sel_trig].param.spd_dj, dt);
#if 0//逆解直接计算角度 全局位置fail
	converV_n_to_b(vmc[leg_sel_trig].param.tar_epos_n.x, vmc[leg_sel_trig].param.tar_epos_n.y, vmc[leg_sel_trig].param.tar_epos_n.z,
		&vmc[leg_sel_trig].param.tar_epos_b.x, &vmc[leg_sel_trig].param.tar_epos_b.y, &vmc[leg_sel_trig].param.tar_epos_b.z);

	converV_b_to_leg(leg_sel_trig, vmc[leg_sel_trig].param.tar_epos_b.x, vmc[leg_sel_trig].param.tar_epos_b.y, vmc[leg_sel_trig].param.tar_epos_b.z,
		&vmc[leg_sel_trig].tar_epos_h.x, &vmc[leg_sel_trig].tar_epos_h.y, &vmc[leg_sel_trig].tar_epos_h.z);

	vmc[leg_sel_trig].tar_epos_h.x = LIMIT(vmc[leg_sel_trig].tar_epos_h.x, MIN_X, MAX_X);//位置限制幅度  输出到IMP摆动控制
	vmc[leg_sel_trig].tar_epos_h.z = LIMIT(vmc[leg_sel_trig].tar_epos_h.z, MAX_Z, MIN_Z);//位置限制幅度

	inv_end_state_new(&vmc[leg_sel_trig],//已经转换内部调用函数
		vmc[leg_sel_trig].tar_epos_h.x,
		vmc[leg_sel_trig].tar_epos_h.y,
		vmc[leg_sel_trig].tar_epos_h.z,
		&vmc[leg_sel_trig].tar_sita1,
		&vmc[leg_sel_trig].tar_sita2,
		&vmc[leg_sel_trig].tar_sita3);
#elif 0//逆解直接计算角度 机体速度good
	inv_end_state_new(&vmc[leg_sel_trig],//已经转换内部调用函数
		vmc[leg_sel_trig].tar_epos_h.x,
		vmc[leg_sel_trig].tar_epos_h.y,
		vmc[leg_sel_trig].tar_epos_h.z,
		&vmc[leg_sel_trig].tar_sita1,
		&vmc[leg_sel_trig].tar_sita2,
		&vmc[leg_sel_trig].tar_sita3);
#elif !SWING_USE_SPD_MODE//雅克比直接换算角度输出goodwithnofb
	espd_to_qspd(&vmc[leg_sel_trig], vmc[leg_sel_trig].epos_spdd_b, vmc[leg_sel_trig].param.spd_dj, dt);

	vmc[leg_sel_trig].tar_sita1 += vmc[leg_sel_trig].param.spd_dj[0] * k_spd_err*dt; 
	vmc[leg_sel_trig].tar_sita2 += vmc[leg_sel_trig].param.spd_dj[1] * k_spd_err*dt;
	vmc[leg_sel_trig].tar_sita3 += vmc[leg_sel_trig].param.spd_dj[2] * k_spd_err*dt;
#endif
}

void reset_tar_pos(char leg_sel_trig) {//区分着地离地？<<----------------------------!!!!!!!!!!!!!!!!
	vmc[leg_sel_trig].param.tar_epos_n = vmc[leg_sel_trig].epos_n;
	vmc[leg_sel_trig].param.tar_epos_b = vmc[leg_sel_trig].epos_b;//摆动N和B系轨迹  Trot

	vmc[leg_sel_trig].param.tar_epos_h = vmc[leg_sel_trig].epos;//H系  阻抗位置模式的角度  复位阻抗
	vmc[leg_sel_trig].param.tar_epos_h_reg = vmc[leg_sel_trig].epos;//之前赋值反了导致着地后跳变Bug0001  unuse now

	vmc[leg_sel_trig].tar_epos_h_reg = vmc[leg_sel_trig].tar_epos_h = vmc[leg_sel_trig].epos;//站立 末端逆解和摆动逆解 规划结果纯 需要滤波到H输出 nouse

	vmc[leg_sel_trig].param.spd_dj[0] = vmc[leg_sel_trig].param.spd_dj[1] = vmc[leg_sel_trig].param.spd_dj[2] = 0;
}

void trig_plan_online(char leg_sel_trig, float dt) {
	cal_tar_end_pos_n_v1_sel(leg_sel_trig, 1, dt); 
	vmc[leg_sel_trig].tar_pos = vmc[leg_sel_trig].tar_epos_n; 
#if EN_PLAN_USE_JERK
	swing_jerk_planner_5point(&vmc[leg_sel_trig], lf_spd, td_spd, vmc_all.gait_time[1] - vmc_all.stance_time);
#else  
	cal_curve_from_pos_new(&vmc[leg_sel_trig], vmc[leg_sel_trig].tar_pos, vmc_all.gait_time[1] - vmc_all.stance_time - lf_time);
#endif	
}