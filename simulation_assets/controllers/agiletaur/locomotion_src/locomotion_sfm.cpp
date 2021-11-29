#include "include.h"
#include "locomotion_header.h"
#include "gait_math.h"
#include "can.h"
#include "eso.h"
#if !RUN_WEBOTS
#include "beep.h"
#endif

Gait_Mode gait_ww;
POS_FORCE_PARM pos_force_p;
VMC_ROBOT_PARM vmc_robot_p;
VMC vmc[4];
VMC_ALL vmc_all;
ESO att_rate_eso[3];
_OCU ocu;
_SDK sdk;
 

float MIN_Z = -0.1;
float MAX_Z = -0.19;
float MIN_Y = -0.1;
float MAX_Y = -0.1;
float MIN_X = -0.15;
float MAX_X = 0.15;
float MAX_SPD = 0;
float MAX_SPD_RAD = 50;
char state_pass = 1;
char test_pos_st = 0;//测试标志位
float test_st_exp[5] = { 15,10,0.04,0.03,1.5 };//P R X Z SPD
char stand_force_enable_flag[5] = { 0 };

void vmc_param_init(void)
{
	int i, j;
	float scale_x = 1;
	float scale_z = 20;
	float pos_gain_scale_st = 1;
	float pos_gain_scale_sw = 1;

	for (i = 0; i < 4; i++) {
		vmc[i].param.id = i;
	}
	//------------------------机械参数-------------------------
	vmc_all.l1 = L1w;
	vmc_all.l2 = L2w;
	vmc_all.l3 = L3w;
	vmc_all.H = Hw;
	vmc_all.W = Www;
	vmc_all.param.robot_type = MOCO8_MID;
	vmc_all.param.leg_dof = 2;
	vmc_all.param.leg_type = PARALE;
	vmc_all.gait_mode = IDLE;
	vmc_all.mess = Mw;

	vmc_all.use_ground_sensor_new = EN_GROUND_CHECK;
	vmc_all.tar_att_bias[PITr] = vmc_all.tar_att_bias[ROLr] = 0;
	vmc_all.param.param_vmc_default.stance_time[1] = vmc_all.param.param_vmc_default.stance_time[0];

	vmc_all.param.safe_sita[0] = -35;//复位角度
	vmc_all.param.safe_sita[1] = 180 + 35;
	vmc_all.param.safe_sita[2] = 0;
	memcpy(&vmc_all.param.param_vmc, &vmc_all.param.param_vmc_default, sizeof(vmc_all.param.param_vmc_default));

	odom1.Kpv = 32;//16;
	//--------------------------------------------------------
	vmc_all.gait_mode = TROT;

	if (vmc_all.param.leg_type == LOOP || vmc_all.param.leg_type == PARALE_LOOP) {
		MIN_Z = -sind(15)*vmc_all.l1 - sind(15)*vmc_all.l2;//0.04;
		MAX_Z = -sind(50)*vmc_all.l1 - sind(50)*vmc_all.l2;//0.09;
		MIN_Y = -sind(40)*(vmc_all.l1 + vmc_all.l2)*0.8;//0.06;
		MAX_Y = sind(40)*(vmc_all.l1 + vmc_all.l2)*0.8;
		MIN_X = -sind(45)*vmc_all.l2;
		MAX_X = sind(45)*vmc_all.l2;
	}
	else {//并联
		MIN_Z = -(cosd(40)*vmc_all.l2 - cosd(60)*vmc_all.l1);
		MAX_Z = -(sind(66)*vmc_all.l1 + sind(70)*vmc_all.l2);

		MIN_X = -vmc_all.l1*1.15;
		MAX_X = vmc_all.l1*1.15;
		MIN_Y = vmc_all.l1*1.15;
		MAX_Y = vmc_all.l1*1.15;
	}

	float temp[10];
	temp[0] = cosd(45)*vmc_all.l1 + cosd(45)*vmc_all.l2;
	temp[1] = cosd(65)*vmc_all.l1 + cosd(65)*vmc_all.l2;

	if (vmc_all.param.leg_dof == 3) {
		vmc_all.param.MAX_PIT = LIMIT(atan2(temp[0] - temp[1], vmc_all.H)*57.3*0.8, -25, 25);
		vmc_all.param.MAX_ROL = LIMIT(atan2(temp[0] - temp[1], vmc_all.W)*57.3*0.8, -25, 25);
	}
	else {
		vmc_all.param.MAX_PIT = LIMIT(atan2(temp[0] - temp[1], vmc_all.H)*57.3*0.6, -25, 25);
		vmc_all.param.MAX_ROL = LIMIT(atan2(temp[0] - temp[1], vmc_all.W)*57.3*0.4, -25, 25);
	}

	temp[0] = cosd(45)*vmc_all.l1;
	temp[1] = sind(45)*vmc_all.l1;
	vmc_all.param.MAX_YAW = LIMIT(90 - atan2(vmc_all.H + 2 * temp[1], 2 * temp[0])*57.3, -45, 45);

	att_rate_eso[ROLr].b0 = att_rate_eso[PITr].b0 = att_rate_eso[YAWr].b0 = 1;
	vmc_all.param.end_sample_dt = 0.01 * 0;

	vmc[FL1].flag_rl = vmc[BL1].flag_rl = 1;
	vmc[FL2].flag_rl = vmc[BL2].flag_rl = -1;
	vmc[FL1].flag_fb = vmc[FL2].flag_fb = 1;
	vmc[BL1].flag_fb = vmc[BL2].flag_fb = -1;

	if (vmc_all.param.leg_type == LOOP || vmc_all.param.leg_type == PARALE_LOOP)
	{
		vmc_all.sita_test[1] = 90; vmc_all.sita_test[0] = 90;
	} 
	else
	{
		vmc_all.sita_test[1] = 180; vmc_all.sita_test[0] = 0;
	} 

	if (vmc_all.param.leg_type == LOOP || vmc_all.param.leg_type == PARALE_LOOP)
		MAX_SPD = LIMIT(MAX_X / (0.4 + 1e-6)*1.5, 0.01, MAX_FSPD);
	else
		MAX_SPD = LIMIT(vmc_all.l1 * 2 / (vmc_all.gait_time[0] + 1e-6), 0.01, MAX_FSPD);

	for (i = 0; i < 4; i++) {
		vmc[i].l1 = vmc_all.l1; vmc[i].l2 = vmc_all.l2; vmc[i].l3 = vmc_all.l3; vmc[i].l4 = vmc_all.l4;
		vmc[i].sita1 = 180; vmc[i].sita2 = 0;
		vmc[i].ground = 0;
		vmc[i].param.trig_state = 99;
	}
	//----------------------------------------------------------------------------------------
	for (i = 0; i < 4; i++)
	{
		robotwb.Leg[i].id = i;
		robotwb.Leg[i].limit_sita[0] = 45;
		robotwb.Leg[i].limit_sita[1] = 10;
		robotwb.Leg[i].limit_tao[0] = 5;
		robotwb.Leg[i].limit_tao[1] = 5;
		robotwb.Leg[i].is_ground = 0;
		robotwb.Leg[i].delta_ht = 0.03;
		robotwb.Leg[i].ground_state = 0;
		robotwb.Leg[i].time_trig = 0;
		robotwb.Leg[i].cnt_ss = 0;
		robotwb.Leg[i].is_touch_est = 0;
	}

	robotwb.gait_time = 0.5;
	robotwb.stance_time = robotwb.gait_time / 2;

	robotwb.Leg[0].flag_fb = robotwb.Leg[2].flag_fb = 1;
	robotwb.Leg[1].flag_fb = robotwb.Leg[3].flag_fb = -1;
	robotwb.Leg[0].flag_rl = robotwb.Leg[1].flag_rl = 1;
	robotwb.Leg[2].flag_rl = robotwb.Leg[3].flag_rl = -1;

	robotwb.MIN_Z = -(cosdw(40)*L2w - cosdw(60)*L1w);
	robotwb.MAX_Z = -(cosdw(25)*L1w + cosdw(25)*L2w);

	robotwb.MIN_X = -L1w * 1.5;
	robotwb.MAX_X = L1w * 1.5;

	robotwb.vect3_zero.x = robotwb.vect3_zero.y = robotwb.vect3_zero.z = 0;

	gait_ww.state_gait = 0;

#if !defined(CAN_ANL_MIT_MODE)
	pos_gain_scale_sw = 0.7;
	pos_gain_scale_st = 0.7;
#endif

	//----------------------------底层伺服参数--------------------------------
	for (i = 0; i < 4; i++)
	{
		//--------------------------------关节PD------------------------------------------	
		robotwb.Leg[i].q_pid_sw.kp = pos_force_p.q_pid_sw.kp = 0.17 / (10 / 1000.);//pos gain  关节闭环
		robotwb.Leg[i].q_pid_sw.ki = pos_force_p.q_pid_sw.ki = 0.00 / (10 / 1000.);
		robotwb.Leg[i].q_pid_sw.kd = pos_force_p.q_pid_sw.kd = 0.005 / (10 / 1000.);
		robotwb.Leg[i].q_pid_sw.vff = pos_force_p.q_pid_sw.vff = 45;

		robotwb.Leg[i].q_pid_st.kp = pos_force_p.q_pid_st_stance.kp = 0.17 / (10 / 1000.);//       力控关节增益 Stand
		robotwb.Leg[i].q_pid_st.ki = pos_force_p.q_pid_st_stance.ki = 0.00 / (10 / 1000.);
		robotwb.Leg[i].q_pid_st.kd = pos_force_p.q_pid_st_stance.kd = 0.005 / (10 / 1000.);
		robotwb.Leg[i].q_pid_st.vff = pos_force_p.q_pid_st_stance.vff = 0;

		pos_force_p.q_pid_st_trot = pos_force_p.q_pid_st_stance;
		//--------------------------------IMP单腿阻抗------------------------------------------
		robotwb.Leg[i].f_pos_pid_st[Xr].kp = pos_force_p.f_pos_pid_st[Xr].kp = 0.05;//
		robotwb.Leg[i].f_pos_pid_st[Xr].kd = pos_force_p.f_pos_pid_st[Xr].kd = 0.0001;//

		robotwb.Leg[i].f_pos_pid_st[Zr].kp = pos_force_p.f_pos_pid_st[Zr].kp = 0.15;//
		robotwb.Leg[i].f_pos_pid_st[Zr].kd = pos_force_p.f_pos_pid_st[Zr].kd = 0.0001;
		//-----------------------------------------------------------------------------
		robotwb.Leg[i].f_pid_st[Xr].kp = pos_force_p.f_pid_st[Xr].kp = 0.068;
		robotwb.Leg[i].f_pid_st[Zr].kp = pos_force_p.f_pid_st[Zr].kp = 0.068;

		vmc[i].param.invert_knee_epos[Xr] = 1;

		//关节刚度不够姿态动腿会内收  Trot走不起来 
		//位置刚度不够姿态动会前后晃动 
		//摆动关节刚度太大容易摆动中着地
	}
#if RUN_WEBOTS&&!RUN_PI
	//-----------------------robot p  全局参数伺服
	//---------------------VMC 全局参数
	pos_force_p.load_fz = 3;//N Load力
	pos_force_p.td_before_fz = 3;
	pos_force_p.motor_i_b[1] = pos_force_p.motor_i_b[0] = 0.000;
	vmc_robot_p.ground_mu = 0.56;

	vmc_robot_p.stand_off.x = 0.0;//站立落足固定偏差
	vmc_all.param.sw_com_off[Xr] = -vmc_robot_p.stand_off.x;
	vmc_all.param.cof_off_all[F] = 0;
	vmc_all.param.cof_off_all[B] = -0.01;

	vmc_robot_p.att_pit.kp = 400;//PIT姿态 ST
	vmc_robot_p.att_pit.ki = 0.5;
	vmc_robot_p.att_pit.kd = 30;

	vmc_robot_p.att_rol.kp = 700;//ROL姿态
	vmc_robot_p.att_rol.ki = 0.5;
	vmc_robot_p.att_rol.kd = 30;

	vmc_robot_p.att_pit_trot.kp = 400;//PIT姿态
	vmc_robot_p.att_pit_trot.ki = 0.5;
	vmc_robot_p.att_pit_trot.kd = 20.0;

	vmc_robot_p.att_rol_trot.kp = 700;//ROL姿态
	vmc_robot_p.att_rol_trot.ki = 0.5;
	vmc_robot_p.att_rol_trot.kd = 30.0;

	vmc_robot_p.att_yaw_trot.kp = 35.5;//YAW姿态
	vmc_robot_p.att_yaw_trot.ki = 0.5;
	vmc_robot_p.att_yaw_trot.kd = 0.05;
	vmc_robot_p.att_yaw_trot.vff = 15;

	vmc_robot_p.pos_x.kp = 1000;//X站立 Stand
	vmc_robot_p.pos_x.ki = 10;
	vmc_robot_p.pos_x.kd = 100;

	vmc_robot_p.spd_x.kp = 400;//X速度 Trot >100抖动
	vmc_robot_p.spd_x.ki = 0;
	vmc_robot_p.spd_x.kd = 10;

	vmc_robot_p.pos_z.kp = 3500;//Z高度刚度
	vmc_robot_p.pos_z.ki = 50;
	vmc_robot_p.pos_z.kd = 350;

	vmc_robot_p.mess_scale = 1.0;//质量前馈比例

	robotwb.max_torque.x = 15;//ROL  全局姿态扭矩
	robotwb.max_torque.y = 15;//PIT
	robotwb.max_torque.z = 2.5;//YAW
	robotwb.max_force.x = 210;//N 全局最大力
	robotwb.max_force.z = 300;//N
	robotwb.max_err_force.x = 35;
	robotwb.max_err_force.z = 35;//WS//太小导致站立都有问题！！！！ 《10不行

	//着地判断阈值new
	pos_force_p.touch_z_param_st.st_td = 3;//N  站立
	pos_force_p.touch_z_param_st.st_lf = 1;//N

	pos_force_p.touch_z_param_trot_st.st_td = 2;//N  TROT
	pos_force_p.touch_z_param_trot_st.st_lf = 1;//N

	pos_force_p.touch_z_param_bound_st.st_td = 10;//N  BOUND
	pos_force_p.touch_z_param_bound_st.st_lf = 3;//N

	pos_force_p.touch_z_param_sw.st_td = 20;//N  SW
	pos_force_p.touch_z_param_sw.st_lf = 3;//N

	pos_force_p.touch_z_param_td.st_td = 17;//N  TD
	pos_force_p.touch_z_param_td.st_lf = 3;//N

	pos_force_p.touch_z_param_st.check_spd = 0.5;
	pos_force_p.touch_z_param_trot_st.check_spd = 0.5;
	pos_force_p.touch_z_param_bound_st.check_spd = 0.5;
	pos_force_p.touch_z_param_sw.check_spd = 1.0;
	pos_force_p.touch_z_param_td.check_spd = 1.0;

	pos_force_p.touch_z_param_st.check_td = 1;
	pos_force_p.touch_z_param_st.check_lf = 1;
	pos_force_p.touch_z_param_trot_st.check_td = 1;
	pos_force_p.touch_z_param_trot_st.check_lf = 1;
	pos_force_p.touch_z_param_bound_st.check_td = 1;
	pos_force_p.touch_z_param_bound_st.check_lf = 1;
	pos_force_p.touch_z_param_sw.check_td = 2;
	pos_force_p.touch_z_param_sw.check_lf = 1;
	pos_force_p.touch_z_param_td.check_td = 2;
	pos_force_p.touch_z_param_td.check_lf = 1;

	//最大扭矩矩和最大电流
	pos_force_p.max_i = 50;//A
	pos_force_p.max_t = 21;//N/m

	pos_force_p.en_force_control_cal = USE_FPOS_CONTROL;
	pos_force_p.en_force_control_out = USE_FPOS_CONTROL;
	//---------------------摆动参数
	vmc_all.param.param_vmc_default.swing_spdkp[0] = 0.03;//速度反馈
	vmc_all.param.param_vmc_default.stance_time[0] = 0.4;// 0.368;//摆动时间
	vmc_robot_p.sw_deltah = 0.05;//摆动高度
	vmc_all.delay_time[0] = 0.225*0;//摆动延时

	vmc_all.param.param_vmc_default.ground_seek_spd = 0.01;//m/s非着地下探速度
#else
	//-----------------------robot p  全局参数伺服
	//---------------------VMC 全局参数
	pos_force_p.load_fz = 1.5;//N Load力
	pos_force_p.td_before_fz = 6;
	vmc_robot_p.ground_mu = 0.38;

	vmc_robot_p.stand_off.x = 0.01;//站立落足固定偏差
	vmc_all.param.sw_com_off[Xr] = -vmc_robot_p.stand_off.x;
	vmc_all.param.cof_off_all[F] = 0;
	vmc_all.param.cof_off_all[B] = -0.01;

	vmc_robot_p.att_pit.kp = 30;//PIT姿态 ST
	vmc_robot_p.att_pit.ki = 5;
	vmc_robot_p.att_pit.kd = 1.5;

	vmc_robot_p.att_rol.kp = 20;//ROL姿态
	vmc_robot_p.att_rol.ki = 5;
	vmc_robot_p.att_rol.kd = 0.5;

	vmc_robot_p.att_pit_trot.kp = 25;//PIT姿态
	vmc_robot_p.att_pit_trot.ki = 5.0;
	vmc_robot_p.att_pit_trot.kd = 1.0;

	vmc_robot_p.att_rol_trot.kp = 20;//ROL姿态
	vmc_robot_p.att_rol_trot.ki = 5.0;
	vmc_robot_p.att_rol_trot.kd = 0.5;

	vmc_robot_p.att_yaw_trot.kp = 2.0;//YAW姿态
	vmc_robot_p.att_yaw_trot.ki = 0.68;
	vmc_robot_p.att_yaw_trot.kd = 0.068;
	vmc_robot_p.att_yaw_trot.vff = 0.8;

	vmc_robot_p.pos_x.kp = 200;//X站立 Stand
	vmc_robot_p.pos_x.ki = 80;
	vmc_robot_p.pos_x.kd = 40;

	vmc_robot_p.spd_x.kp = 120;//X速度 Trot >100抖动
	vmc_robot_p.spd_x.ki = 50;
	vmc_robot_p.spd_x.kd = 0.05;

	vmc_robot_p.pos_z.kp = 800;//Z高度刚度
	vmc_robot_p.pos_z.ki = 100;
	vmc_robot_p.pos_z.kd = 60;

	vmc_robot_p.mess_scale = 1.0;//质量前馈比例

	robotwb.max_torque.x = 8.8;//ROL  全局姿态扭矩
	robotwb.max_torque.y = 8.6;//PIT
	robotwb.max_torque.z = 2.5;//YAW
	robotwb.max_force.x = 30;//N 全局最大力
	robotwb.max_force.z = 60;//N
	robotwb.max_err_force.x = 3;
	robotwb.max_err_force.z = 3;//WS//太小导致站立都有问题！！！！ 《10不行

	//着地判断阈值new
	pos_force_p.touch_z_param_st.st_td = 0.5;//N  站立
	pos_force_p.touch_z_param_st.st_lf = 0.25;//N
	pos_force_p.touch_z_param_st.check_spd = 0.15;

	pos_force_p.touch_z_param_trot_st.st_td = 0.8;//N  TROT
	pos_force_p.touch_z_param_trot_st.st_lf = 0.45;//N
	pos_force_p.touch_z_param_trot_st.check_spd = 0.15;

	pos_force_p.touch_z_param_sw.st_td = 4.7;//N  SW
	pos_force_p.touch_z_param_sw.st_lf = 0.4;//N
	pos_force_p.touch_z_param_sw.check_spd = 0.15;

	pos_force_p.touch_z_param_td.st_td = 1.2;//N  TD
	pos_force_p.touch_z_param_td.st_lf = 0.4;//N
	pos_force_p.touch_z_param_td.check_spd = 0.3;

	pos_force_p.touch_z_param_st.check_td = 2;
	pos_force_p.touch_z_param_st.check_lf = 1;
	pos_force_p.touch_z_param_sw.check_td = 3;
	pos_force_p.touch_z_param_sw.check_lf = 1;
	pos_force_p.touch_z_param_td.check_td = 2;
	pos_force_p.touch_z_param_td.check_lf = 1;

	//最大扭矩矩和最大电流
	pos_force_p.max_i = 30;//A
	//pos_force_p.max_i=2;//A  for safe test
	pos_force_p.max_t = 3.0;//N/m

	pos_force_p.en_force_control_cal = USE_FPOS_CONTROL;
	pos_force_p.en_force_control_out = USE_FPOS_CONTROL;
	//---------------------摆动参数
	vmc_all.param.param_vmc_default.swing_spdkp[0] = 0.03;//速度反馈
	vmc_all.param.param_vmc_default.stance_time[0] = 0.38;//摆动时间
	vmc_robot_p.sw_deltah = 0.043;//摆动高度
	vmc_all.delay_time[0] = 0.05*TEST_TROT_SW;//摆动延时

	vmc_all.param.param_vmc_default.ground_seek_spd = 0.01;//m/s非着地下探速度

#if TEST_TROT_SW&&0
	vmc_all.param.param_vmc_default.stance_time[0] = 0.68;//摆动时间
#endif
#endif
	vmc_all.gait_time[0] = vmc_all.gait_time[1] = vmc_all.param.param_vmc_default.stance_time[0]; 
	vmc_all.gait_alfa = 0.5;
	vmc_all.stance_time = vmc_all.gait_time[1] * vmc_all.gait_alfa;
	vmc_all.gait_delay_time = vmc_all.stance_time * 0;
}

void vmc_reset(void)
{
	int i = 0;
	vmc_all.tar_att[YAWr] = vmc_all.att_ctrl[YAWr];
	for (i = 0; i < 4; i++)
	{
		vmc[i].spd.x = vmc[i].spd.y = vmc[i].spd.z = 0;
		vmc[i].spd_b.x = vmc[i].spd_b.y = vmc[i].spd_b.z = 0;
		vmc[i].spd_n.x = vmc[i].spd_n.y = vmc[i].spd_n.z = 0;
		vmc[i].param.trig_state = vmc[i].ground = 0;
	}
	vmc_all.ground_num = 0;
	vmc_all.att_trig[0] = vmc_all.att_ctrl[0] = vmc_all.att[0];
	vmc_all.att_trig[1] = vmc_all.att_ctrl[1] = vmc_all.att[1];
	vmc_all.att_trig[2] = vmc_all.att_ctrl[2] = vmc_all.att[2];
	vmc_all.att_vm_b[PITr] = vmc_all.att_vm_b[ROLr] = vmc_all.att_vm_b[YAWr] = 0;
	vmc_all.att_rate_vm[PITr] = vmc_all.att_rate_vm[ROLr] = vmc_all.att_rate_vm[YAWr] = 0;
	vmc_all.att_rate_ctrl[PITr] = vmc_all.att_rate_ctrl[ROLr] = vmc_all.att_rate_ctrl[YAWr] = 0;

	vmc_all.pos.z = 0;
	vmc_all.pos_n.z = 0;
	vmc_all.body_spd[Xr] = vmc_all.body_spd[Yr] = vmc_all.body_spd[Zr] = 0;
	vmc_all.spd_n.x = vmc_all.spd_n.y = vmc_all.spd_n.z = 0;
	vmc_all.param.encoder_spd[Ls] = vmc_all.param.encoder_spd[R] = 0;
	vmc_all.body_spd[YAWrr] = vmc_all.att_ctrl[YAWr];
	vmc_all.param.cog_off_use[3] = 0;
}

char safe_check(float dt)
{
	char i = 0;
	static float timer_q_safe[4] = { 0 };
#if !TEST_TROT_SW
	if (fabs(robotwb.exp_att.pitch - robotwb.now_att.pitch) > SAFE_PITCH&&stand_force_enable_flag[4] == 1)
		return 1;
	if (fabs(robotwb.exp_att.roll - robotwb.now_att.roll) > SAFE_ROLL&&stand_force_enable_flag[4] == 1)
		return 1;
#endif
	for (i = 0; i < 4; i++) {//∠角度保护1
		if (fabs(To_180_degreesw(vmc[i].sita1 - vmc[i].sita2)) < 5 && stand_force_enable_flag[4] == 1)
			timer_q_safe[i] += dt;
		else
			timer_q_safe[i] = 0;

		if (timer_q_safe[i] > 2) {
			timer_q_safe[i] = 0;
			return 1;
		}
	}
	return 0;
}

//--------------------------------------------------主步态状态机------------------------------------------------
float qd_init[2] = { 88, 88 };//初始化角速度
float qd_reset[2] = { -70,-70 };
char sit_down_flag = 0;
void locomotion_sfm(float dt)
{
	int i, j;
	char robot_protect;
	static char temp_q_rst_flag = 0, temp_err_rst_flag = 0;
	char temp_flag[4] = { 0 };
	static int init = 0;
	static char ocu_key_ud_reg;
	static float timer[10] = { 0 };
	if (!init) {
		init = 1;
		vmc_param_init();
	}

	switch (gait_ww.state_gait)
	{
#if !RUN_WEBOTS||RUN_PI
	case 0:
		switch (temp_q_rst_flag)
		{
		case 0:
			for (i = 0; i < 4; i++) {//电机状态正确 角度显示用
				robotwb.Leg[i].tao_q_i[0] = robotwb.Leg[i].tao_q_i[1] = 0;//清除角度闭环积分

				if (leg_motor[i].ready[0] == 1 && leg_motor[i].ready[1] == 1 && leg_motor[i].connect_motor[0] == 1 && leg_motor[i].connect_motor[1] == 1)//电机准备好
					vmc[i].param.q_now_use_tar = 0;
				else
					vmc[i].param.q_now_use_tar = 1;
			}

			//-------------------标定角度 按键Y
			if (ocu.key_y)timer[1] += dt; else timer[1] = 0;

			if (timer[1] > 1)
			{
				temp_q_rst_flag++;
				for (i = 0; i < 4; i++)
					leg_motor[i].reset_q = 2;
				robotwb.beep_state = BEEP_BLDC_ZERO_CAL;
				timer[1] = 0;
			}
			break;
		case 1://
			if (!ocu.key_y)timer[1] += dt;

			if (timer[1] > 0.5)
			{
				temp_q_rst_flag = 0;
				for (i = 0; i < 4; i++)
					leg_motor[i].reset_q = 0;
				timer[1] = 0;
			}
			break;
		}	//----		

		//-------------------复位故障
		switch (temp_err_rst_flag)//按键B
		{
		case 0:
			if (ocu.key_b)timer[2] += dt; else timer[2] = 0;

			if (timer[2] > 1)
			{
				temp_err_rst_flag++;
				robotwb.beep_state = BEEP_BLDC_RESET_ERR;
				for (i = 0; i < 4; i++)
					leg_motor[i].motor_en = 0;
				timer[2] = 0;
				leg_motor[0].reset_err = leg_motor[1].reset_err = leg_motor[2].reset_err = leg_motor[3].reset_err = 1;//reset_err_flag=1;
			}
			break;
		case 1:
			if (!ocu.key_b)timer[2] += dt;

			if (timer[2] > 0.5)
			{
				temp_err_rst_flag = 0;
				leg_motor[0].reset_err = leg_motor[1].reset_err = leg_motor[2].reset_err = leg_motor[3].reset_err = 0;//reset_err_flag=0;
				timer[2] = 0;
			}
			break;
		}//-----------			

		//-------------------------驱动器状态检测与角度复位
		if (ocu.key_x)timer[0] += dt; else timer[0] = 0;

		if (timer[0] > 1)
		{
			for (i = 0; i < 4; i++) {
				if (leg_motor[i].ready[0] == 1 && leg_motor[i].ready[1] == 1) {//电机准备好
					leg_motor[i].motor_en = 1;//开始能
					vmc[i].param.q_now_use_tar = 0;
				}
				else {
					leg_motor[i].motor_en = 0;
					vmc[i].param.q_now_use_tar = 1;
				}
			}

			timer[0] = 0; vmc_all.gait_mode = IDLE; vmc_all.power_state = 2; ocu.connect = 1; ocu.mode = 2;
			if (!ocu.key_rr)//按住R进入直接恢复角度不旋转
				gait_ww.state_gait++;
			else
				gait_ww.state_gait += 2;
			sit_down_flag = 0;//进站立标志位
			stand_force_enable_flag[0] = 0; stand_force_enable_flag[1] = 0;
			stand_force_enable_flag[2] = 0; stand_force_enable_flag[3] = 0;
			stand_force_enable_flag[4] = 0;
			robotwb.beep_state = BEEP_BLDC_ZERO_INIT;
			for (i = 0; i < 4; i++) {//复位状态
				vmc[i].ground = 0;//着地清零
				vmc[i].param.trig_state = 0;
				reset_tar_pos(i);
				robotwb.Leg[i].tar_sita[0] = vmc[i].tar_sita1 = vmc[i].sita1;
				robotwb.Leg[i].tar_sita[1] = vmc[i].tar_sita2 = vmc[i].sita2;
			}
		}
		break;
		//----------------------初始化至角度 复位用 自动旋转
	case 1:
		timer[0] += dt;
		for (i = 0; i < 4; i++) {
			switch (i) {
			case 1:case 3:
				move_joint_with_spd(&vmc[i], 0, vmc[i].flag_fb*qd_init[0], dt);
				move_joint_with_spd(&vmc[i], 1, vmc[i].flag_fb*qd_init[1], dt);
				break;
			case 2:case 0:
				move_joint_with_spd(&vmc[i], 0, vmc[i].flag_fb*qd_init[0], dt);
				move_joint_with_spd(&vmc[i], 1, vmc[i].flag_fb*qd_init[1], dt);
				break;
			}
		}

		if ((timer[0] > 3.2 || ocu.key_rr) && state_pass) {//可强制结束 RR
			gait_ww.state_gait++;
			timer[0] = 0;
			for (i = 0; i < 4; i++) {//复位状态
				vmc[i].ground = 0;//着地清零
				vmc[i].param.trig_state = 0;
				reset_tar_pos(i);
				robotwb.Leg[i].tar_sita[0] = vmc[i].tar_sita1 = vmc[i].sita1;
				robotwb.Leg[i].tar_sita[1] = vmc[i].tar_sita2 = vmc[i].sita2;
			}
		}
		break;//----------------------初始化至站立角度		
	case 2:
		for (i = 0; i < 4; i++) {
			temp_flag[i] = 0;
			temp_flag[i] += move_joint_to_pos1(&vmc[i], 0, -35 * USE_FPOS_CONTROL * 1, 180, 1, dt);
			temp_flag[i] += move_joint_to_pos1(&vmc[i], 1, 180 + 35 * USE_FPOS_CONTROL * 1, 180, 1, dt);
			robotwb.Leg[i].tar_sita[0] = vmc[i].tar_sita1;
			robotwb.Leg[i].tar_sita[1] = vmc[i].tar_sita2;
		}

		if (temp_flag[0] == 2 && temp_flag[1] == 2 && temp_flag[2] == 2 && temp_flag[3] == 2)
			timer[0] += dt;

		if (timer[0] > 0.5&&state_pass) {//可强制结束
			gait_ww.state_gait++;
			for (i = 0; i < 4; i++) {//复位状态
				vmc[i].ground = 0;//着地清零
				vmc[i].param.trig_state = 0;
				reset_tar_pos(i);
				robotwb.Leg[i].tar_sita[0] = vmc[i].tar_sita1 = vmc[i].sita1;
				robotwb.Leg[i].tar_sita[1] = vmc[i].tar_sita2 = vmc[i].sita2;
			}
		}
		break;
#else
	case 0://reset joint
		for (i = 0; i < 4; i++) {
			robotwb.Leg[i].tar_sita[0] = -35 * 0;
			robotwb.Leg[i].tar_sita[1] = 180 + 35 * 0;
		}
		timer[0] += dt;
		if (timer[0] > 0.15) {
			vmc_param_init();
			printf("VMC Init Done!\n");
			timer[0] = 0;
			gait_ww.state_gait = 3;
			vmc_all.gait_mode = 0;
			ocu.connect = 1;
			ocu.mode = 2;
			stand_force_enable_flag[0] = 0; stand_force_enable_flag[1] = 0;
			stand_force_enable_flag[2] = 0; stand_force_enable_flag[3] = 0;
			stand_force_enable_flag[4] = 0;
			for (i = 0; i < 4; i++) {//复位状态
				vmc[i].ground = 0;//着地清零
				vmc[i].param.trig_state = 0;
				reset_tar_pos(i);
				robotwb.Leg[i].tar_sita[0] = vmc[i].tar_sita1 = vmc[i].sita1;
				robotwb.Leg[i].tar_sita[1] = vmc[i].tar_sita2 = vmc[i].sita2;
			}
			for (i = 0; i < 4; i++) {
				robotwb.Leg[i].tar_sita[0] = robotwb.Leg[i].sita[0];
				robotwb.Leg[i].tar_sita[1] = robotwb.Leg[i].sita[1];
			}
		}
		pos_control_pd(dt);
		break;
#endif
	case 3://步态线程------------------------<<
		gait_switch(dt);//步态切换状态机

		switch (vmc_all.gait_mode) {
		case STAND_RC:case STAND_IMU:case STAND_PUSH:
			if (test_pos_st || (!EN_AUTO_TORT&&RUN_WEBOTS&&EN_AUTO_ATT_ST))//姿态测试
			{
				timer[3] += test_st_exp[4] / 2.0*dt;

				vmc_all.tar_pos.x = cosdw(timer[3] * 180)*test_st_exp[2] + vmc_robot_p.stand_off.x;
				//vmc_all.tar_pos.z = (fabs(MAX_Z)-fabs(MIN_Z))*0.8 - sindw(timer[3] * 180)*test_st_exp[3];

				vmc_all.tar_att[PITr] = sindw(timer[3] * 180) *  test_st_exp[0];
				vmc_all.tar_att[ROLr] = cosdw(timer[3] * 180) *  test_st_exp[1];
			}
			else {
				vmc_all.tar_pos.x = ocu.rc_spd_w[Xr] * 0.045 + vmc_robot_p.stand_off.x;
				timer[3] = 0;
			}

			Gait_Stand_Update(dt);
			break;
		case PRONK:
			Gait_Pronk_Update(dt);
			break;
		case CLIMB :
			Gait_Climb_Update(dt);
			break;
		case BOUND:
			Gait_Bound_Update(dt);
			break;
		case TROT:case F_TROT:
			vmc_all.unmove = 0;

			if (test_pos_st || EN_AUTO_ATT_TROT)//姿态测试
			{
				timer[3] += test_st_exp[4] / 10.0*dt;

				//vmc_all.tar_pos.x = cosdw(timer[3] * 180)*test_st_exp[2] + vmc_robot_p.stand_off.x;
				//vmc_all.tar_pos.z = (fabs(MAX_Z)-fabs(MIN_Z))*0.8 - sindw(timer[3] * 180)*test_st_exp[3];

				vmc_all.tar_att[PITr] = sindw(timer[3] * 180) *  test_st_exp[0] / 2;
				vmc_all.tar_att[ROLr] = cosdw(timer[3] * 180) *  test_st_exp[1] / 2;
			}
			else {
				vmc_all.tar_pos.x = ocu.rc_spd_w[Xr] * 0.045 + vmc_robot_p.stand_off.x;
				timer[3] = 0;
			}

			Gait_Trot_Update_v1(dt);
			break;
		case RECOVER:case FALLING:
			Gait_Recovery_Update(dt);
			break;
		default:
			break;
		}
		break;
	}
#if !RUN_WEBOTS||RUN_PI
	robot_protect = safe_check(dt);

	if ((ocu.key_ud == -1 && ocu_key_ud_reg == 0) || (robot_protect == 1 && gait_ww.state_gait >= 3)) {//Disable Power 断使能
		vmc_all.tar_spd.x = vmc_all.tar_spd.y = vmc_all.tar_spd.z = 0;
		vmc_all.param.tar_spd_use_rc.x = vmc_all.param.tar_spd_use_rc.y = vmc_all.param.tar_spd_use_rc.z = 0;
		vmc_all.power_state = ocu.cmd_robot_state = vmc_all.leg_power = 0;
		vmc_all.gait_mode = IDLE;
		vmc_all.param.robot_mode = M_SAFE;

		//doghome
		for (i = 0; i < 4; i++) {
#if !RUN_WEBOTS||RUN_PI
			reset_current_cmd(i);
#endif
			leg_motor[i].motor_en = 0;
		}
		gait_ww.state_gait = 0;//reset main state
	}
	ocu_key_ud_reg = ocu.key_ud;
#endif
}


float att_trig_reset_dead = 2.56;
float soft_param[2] = { 0.75,0.6 };//锟斤拷锟斤拷锟劫讹拷  锟斤拷锟斤拷锟斤拷锟斤拷
float z_init_spd = 6.8;//站立初始化速度
float init_end_z = 0;
char gait_switch(float dt)//cushihua  moshi  主步态切换状态机
{
	char i;
	static float timer_webot_auto = 0;
	static char sdk_mode_reg = 0;
	static char stand_switch_trig = 0;
	static float t, t_rst[3], timer[5], t_fall, t_fall_self_right = 0;
	static char state, rst_state, soft_start_state, fall_state, t_um;
	static float sbus_mode_sw_cnt = 0;
	float cog_off, att_off;
	float end_dis[4];
	float att_use[3], err[2];

	att_use[ROLr] = vmc_all.att_ctrl[ROLr];
	err[ROLr] = vmc_all.tar_att[ROLr] + vmc_all.tar_att_off[ROLr] - att_use[ROLr];
	//---------------------------OCU 模式 ------------------------------

	if ((((ocu.connect&&ocu.mode >= 2) || sdk.sdk_mode == 1) && !ocu.sbus_conncect) || (RUN_WEBOTS && !RUN_PI)) { 

		switch (ocu.cmd_robot_state)//moshi
		{
		case 0:
			vmc_all.param.robot_mode = M_SAFE;
			vmc_reset();//
			ocu.cmd_robot_state = 1;
			t_fall_self_right = 0;
			vmc_all.leg_power = vmc_all.unmove = 1;//锟较碉拷
			vmc_all.param.stand_switch_flag[0] = vmc_all.param.stand_switch_flag[1] = 1; 
			vmc_all.param.stand_trot_switch_flag = 0;
			stand_switch_trig = 0;
			printf("Gait LSM Active!\n");
			break;
		case 1: 
			vmc_all.unmove = 1;
			for (i = 0; i < 4; i++) {
				vmc[i].ground = 0;
			}

			if ((RUN_WEBOTS && !RUN_PI) || (ocu.key_ud&&ocu.key_ud_reg == 0) || (ocu.key_ll&&ocu.key_ll_reg == 0) || (ocu.key_rr&&ocu.key_rr_reg == 0)) {//上键切换模式 doghome
				reset_robot_statement();
				vmc_all.power_state = 2;
				ocu.cmd_robot_state = 2;

				for (i = 0; i < 4; i++)
					reset_tar_pos(i);//新增加为位置模式离地的断点初始化

				vmc_all.param.stand_switch_flag[0] = 0; 
				vmc_all.param.stand_switch_flag[1] = 0; 

				vmc_all.param.robot_mode = M_STAND_RC;	vmc_all.gait_mode = STAND_RC;
				Gait_Stand_Active(); 

				robotwb.beep_state = BEEP_BLDC_GAIT_SWITCH;

				if (ocu.key_ll) {
					init_end_z = fabs(MAX_Z)*0.75;
					sit_down_flag = 1;
				}
				else
					init_end_z = fabs(MAX_Z)*0.36;
#if RUN_WEBOTS&&!RUN_PI
				init_end_z = fabs(MAX_Z)*0.75;
				sit_down_flag = 1;
#endif
			}
			break;
		case 2://Stand mode
			cog_off = vmc_all.param.cog_off_use[0];
			att_off = vmc_all.param.cog_off_use[1]; 

			if (ocu.key_ll&&ocu.key_ll_reg == 0) {//下蹬腿   
				if (stand_force_enable_flag[4] == 0)//空中竟可能下探
					init_end_z = fabs(MAX_Z)*0.75;
				else														 //着地步态高度
					init_end_z = fabs(MAX_Z)*0.75;
				sit_down_flag = 1;//下蹬时才触发ST着地判断
			}
			if (ocu.key_rr&&ocu.key_rr_reg == 0)
				init_end_z = fabs(MAX_Z)*0.4;

			vmc_all.tar_pos.z += (init_end_z - vmc_all.tar_pos.z)*z_init_spd*dt;//末端控制
		  //-------------
			vmc_all.param.tar_spd_use_rc.x = vmc_all.param.tar_spd_use_rc.y = vmc_all.param.tar_spd_use_rc.z = 0;
			vmc_all.tar_spd.x = vmc_all.tar_spd.y = vmc_all.tar_spd.z = 0;

			timer_webot_auto += dt;

			if (((ocu.key_a == 1 && ocu.key_a_reg == 0) || (sdk.sdk_mode == 1 && sdk.gait_mode == TROT) || (timer_webot_auto > 3 && RUN_WEBOTS&&EN_AUTO_TORT))
				&& ((fabs(vmc_all.pos.z) > fabs(MIN_Z)*1.3 && !SINGLE_LEG_TEST) || TEST_TROT_SW)) {//Swtich Trot
				ocu.cmd_robot_state = 11;
				Gait_Trot_Active(); 
				vmc_all.param.stand_switch_flag[0] = 0; 
				vmc_all.param.stand_switch_flag[1] = 1;
				vmc_all.param.stand_trot_switch_flag = 1;
				vmc_all.param.stand_switch_cnt[0] = 0;
				stand_switch_trig = 0;
				robotwb.beep_state = BEEP_BLDC_GAIT_SWITCH;
			}

			if((ocu.key_ud == 1 && ocu.key_ud_reg == 0)|| (timer_webot_auto > 3 && RUN_WEBOTS&&EN_AUTO_PRONK))
			{
				ocu.cmd_robot_state = 13;
				Gait_Pronk_Active();
				robotwb.beep_state = BEEP_BLDC_GAIT_SWITCH;
			}

			if ((ocu.key_b == 1 && ocu.key_b_reg == 0) || (timer_webot_auto > 3 && RUN_WEBOTS&&EN_AUTO_CLIMB))
			{
				ocu.cmd_robot_state = 14;
				Gait_Climb_Active(); 
				robotwb.beep_state = BEEP_BLDC_GAIT_SWITCH;
			}

			if ((ocu.key_b == 1 && ocu.key_b_reg == 0) || (timer_webot_auto > 3 && RUN_WEBOTS&&EN_AUTO_BOUND))
			{
				ocu.cmd_robot_state = 15;
				Gait_Bound_Active(); 
				robotwb.beep_state = BEEP_BLDC_GAIT_SWITCH;
			}

			if (fabs(vmc_all.att_ctrl[ROLr]) > 30)
				t_fall_self_right += dt;
			else
				t_fall_self_right = 0;

			if (t_fall_self_right > 0.05 && 0) {
				Gait_Recovery_Active();
				ocu.cmd_robot_state = 12;
			}
			break;
		case 11://Trot		
			if (ocu.key_ll) 
				vmc_all.tar_pos.z += dt * 0.05;
			if (ocu.key_rr)
				vmc_all.tar_pos.z -= dt * 0.05;

			if (sdk.sdk_mode == 1) {
				if (sdk.cmd_z != 0)
					vmc_all.tar_pos.z = sdk.cmd_z;
				else
					vmc_all.tar_pos.z += dt * sdk.cmd_vz;
			}

			vmc_all.tar_pos.z = LIMIT(vmc_all.tar_pos.z, fabs(MIN_Z)*1.2, fabs(MAX_Z)*0.9);

			if (ocu.key_lr == 1 && ocu.key_lr_reg == 0)
			{
				if (vmc_all.gait_mode == TROT && 0) {
					vmc_all.param.robot_mode = M_F_TROT; 
					vmc_all.gait_mode = F_TROT;
				}
				else if (vmc_all.gait_mode == F_TROT) {
					vmc_all.param.robot_mode = M_TROT; 
					vmc_all.gait_mode = TROT;
				}
			}

			if (((ocu.key_b == 1 && ocu.key_b_reg == 0) ||
				(sdk.sdk_mode == 1 && sdk.gait_mode == STAND_IMU)) && stand_switch_trig == 0) 
			{
				stand_switch_trig = 1; 
				vmc_all.param.stand_switch_flag[0] = 1;
				vmc_all.param.stand_switch_flag[1] = 0; 
				vmc_all.param.stand_switch_cnt[1] = 0;
				vmc_all.param.stand_trot_switch_flag = 2;
				robotwb.beep_state = BEEP_BLDC_GAIT_SWITCH;
			}

			if (vmc_all.param.stand_trot_switch_flag == 0 && stand_switch_trig == 1) { 
				stand_switch_trig = 0;
				ocu.cmd_robot_state = 2;
				vmc_all.param.robot_mode = M_STAND_IMU;	vmc_all.gait_mode = STAND_IMU;
				vmc[0].ground = vmc[1].ground = vmc[2].ground = vmc[3].ground = 1;
				Gait_Stand_Active(); 
			}

			if (fabs(vmc_all.att_ctrl[ROLr]) > 23)
				t_fall_self_right += dt;
			else
				t_fall_self_right = 0;

			if (t_fall_self_right > 0.05 && 0) {
				Gait_Recovery_Active();
				ocu.cmd_robot_state = 12;
			}
			break;

		case 12://自恢复

			if (ocu.key_lr == -1 && ocu.key_lr_reg == 0)
			{
				if (vmc_all.param.leg_dof == 3) {
					vmc_all.param.robot_mode = M_RECOVER; 
					vmc_all.gait_mode = RECOVER;
				}
				else {
					vmc_reset();
					ocu.cmd_robot_state = 1;
					t_fall_self_right = 0;
					vmc_all.leg_power = vmc_all.unmove = 1; 
					vmc_all.param.stand_switch_flag[0] = vmc_all.param.stand_switch_flag[1] = 1;
					vmc_all.param.stand_switch_cnt[0]=vmc_all.param.stand_switch_cnt[1]=0;
					vmc_all.param.stand_trot_switch_flag = 0;
					stand_switch_trig = 0;
				}
			}
			break;
		}

		//-------------------------- --------------------------------
		if (((ocu.key_ud == -1 && ocu.key_ud_reg == 0) || (sdk_mode_reg == 1 && sdk.sdk_mode == 0)) && vmc_all.sita_test[4] == 0) {//Disable Power
			vmc_all.tar_spd.x = vmc_all.tar_spd.y = vmc_all.tar_spd.z = 0;
			vmc_all.param.tar_spd_use_rc.x = vmc_all.param.tar_spd_use_rc.y = vmc_all.param.tar_spd_use_rc.z = 0;
			vmc_all.power_state = ocu.cmd_robot_state = vmc_all.leg_power = 0;
			vmc_all.gait_mode = IDLE;
			vmc_all.param.robot_mode = M_SAFE;
			sit_down_flag = 0;//进站立标志位
			stand_force_enable_flag[0] = 0; stand_force_enable_flag[1] = 0;
			stand_force_enable_flag[2] = 0; stand_force_enable_flag[3] = 0;
			stand_force_enable_flag[4] = 0;
			init_end_z = fabs(MAX_Z)*0.36;
			//doghome
			for (i = 0; i < 4; i++) {
				robotwb.Leg[i].tar_force_dis_n.x = robotwb.Leg[i].tar_force_dis_n.z = 0;
				reset_tar_pos(i);
				leg_motor[i].motor_en = 0;
			}
			stand_force_enable_flag[0] = 0;
			stand_force_enable_flag[1] = 0;
			stand_force_enable_flag[2] = 0;
			stand_force_enable_flag[3] = 0;
			stand_force_enable_flag[4] = 0;
			gait_ww.state_gait = 0;//reset main state
		}

		sdk_mode_reg = sdk.sdk_mode; ocu.key_ud_reg = ocu.key_ud; ocu.key_lr_reg = ocu.key_lr; ocu.key_x_reg = ocu.key_x; ocu.key_y_reg = ocu.key_y; ocu.key_a_reg = ocu.key_a;
		ocu.key_b_reg = ocu.key_b; ocu.key_ll_reg = ocu.key_ll; ocu.key_rr_reg = ocu.key_rr; ocu.key_st_reg = ocu.key_st; ocu.key_back_reg = ocu.key_back;
	}


	static char ocu_sbus_conncect_reg = 0;
	//--------------------------- ------------------------------
	if (ocu_sbus_conncect_reg && !ocu.sbus_conncect&&ocu.cmd_robot_state > 1) { 
		vmc_all.tar_spd.x = vmc_all.tar_spd.y = vmc_all.tar_spd.z = 0;
		vmc_all.param.tar_spd_use_rc.x = vmc_all.param.tar_spd_use_rc.y = vmc_all.param.tar_spd_use_rc.z = 0;
		vmc_all.power_state = ocu.cmd_robot_state = vmc_all.leg_power = 0;
		vmc_all.gait_mode = IDLE;
		vmc_all.param.robot_mode = M_SAFE;
	}
	ocu_sbus_conncect_reg = ocu.sbus_conncect;

	switch (fall_state)
	{
	case 0:
		if (fabs(vmc_all.att[PITr]) > 70 || fabs(vmc_all.att[ROLr]) > 70)
			t_fall += dt;
		else
			t_fall = 0;

		if (t_fall > 0.5)
			vmc_all.fall = 1;
		if (vmc_all.fall&&vmc_all.param.en_fall_protect)
		{
			fall_state = 1; t_fall = 0; vmc_all.param.robot_mode = M_FALLING;
		}
		break;
	case 1: 
		vmc_all.tar_spd.x = vmc_all.tar_spd.y = vmc_all.tar_spd.z = 0;
		vmc_all.param.tar_spd_use_rc.x = vmc_all.param.tar_spd_use_rc.y = vmc_all.param.tar_spd_use_rc.z = 0;
		vmc_all.power_state = ocu.cmd_robot_state = vmc_all.leg_power = 0;
		for (i = 0; i < 4; i++) {
			vmc[i].sita1 = vmc_all.param.safe_sita[0];
			vmc[i].sita2 = vmc_all.param.safe_sita[1];
			vmc[i].sita3 = vmc_all.param.safe_sita[2];
			estimate_end_state_new(&vmc[i], dt);
			vmc[i].ground = 1;
		}
		t_fall += dt;
		if (t_fall > 2)
		{
			t_fall = 0; fall_state = 2;
		}
		break;
	case 2:
		vmc_all.tar_spd.x = vmc_all.tar_spd.y = vmc_all.tar_spd.z = 0;
		vmc_all.param.tar_spd_use_rc.x = vmc_all.param.tar_spd_use_rc.y = vmc_all.param.tar_spd_use_rc.z = 0;
		vmc_all.power_state = ocu.cmd_robot_state = vmc_all.leg_power = 0;

		if (fabs(vmc_all.att[PITr]) < 25 && fabs(vmc_all.att[ROLr]) < 25)
			t_fall += dt;
		else
			t_fall = 0;

		if (t_fall > 2)
		{
			t_fall = 0; fall_state = 0; vmc_all.fall = 0;
		}
		break;
	}

	return vmc_all.leg_power;
}