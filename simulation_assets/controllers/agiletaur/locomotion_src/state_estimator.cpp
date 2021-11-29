#include "include.h"
#include "locomotion_header.h"
#include "gait_math.h"
#include "eso.h"
#include "ground_att_est_n.h"
#include "wbInterface.h"
int three_leg = 0, three_leg_sel = 0;
/*              / Z
				|
				|      S0为-
				O------------------>X     力矩方向与速度一致 S0+加 S0力矩正 向下压
			   /  \    S0为+                                S1-加 S1力矩负 向下压
			  /    \
			 /      \
			/ S1     \L1
		   /          \
			   Q_front       Q_back
	leg 0 FR
	leg 1 HR
	leg 2 FL     0
	leg 3 HL
*/
robotTypeDef robotwb;
_ODOM_1 odom1;
float FLT_GRF = 0;
float gait_change_param[6] = { 16.8,0.00063,4,0.0013,0.0732,0.36 };
float FLT_ATT_CTRL[3] = { 6,0.68,0.68 };
float FLT_ATT_TRIG = 15;
float FLT_AUTO_T = 0.5;
float FLT_ENCODER = 2;
float FLT_POLAR = 1;
float FLT_Z_UNMATCH = 5;
float FLT_ATT_CTRL_DIS = 1.68;
float FLT_COG_OFF = 0.5;
float FLT_SPD_END_Z = 8;
float FLT_SPD_END_X_ODOM = 8;
float FLT_POS_Z = 8;
float FLT_POS_X = 8;
static float FLT_ATT_CTRL_Y = 0.35;
float en_fix[5] = { 0,1,1,1,0.85 };
float w_r_vm[2] = { 0.268,1 };
float k_rate_vm[2] = { 0.567,0.1 };
float k_att_trig_cal[2] = { 0,0 };
int acc_flag_walk[2] = { -1,1 };
float acc_n_walk[2];
float att_rate_vm_yaw = 0;
float odom_td_p = 16.8;

//复位机器人状态估计
void reset_robot_statement(void)
{
	char i = 0;
	vmc_all.tar_att[YAWr] = vmc_all.att_ctrl[YAWr];
	vmc_all.gait_time[3] = vmc_all.gait_time[0];
	vmc_all.delay_time[1] = vmc_all.delay_time[0];
	for (i = 0; i < 3; i++) {
		vmc_all.att_vm[i] = vmc_all.att_vm_b[i] = vmc_all.att[i];
		vmc_all.att_rate_vm[i] = 0;

		vmc_all.att_rate_trig[i] = vmc_all.att_rate[i];
		vmc_all.att_rate_ctrl[i] = vmc_all.att_rate[i];
		vmc_all.att_trig[i] = vmc_all.att[i];
		vmc_all.att_ctrl[i] = vmc_all.att[i];
		vmc_all.body_spd[i] = 0;
	}
	vmc_all.att_trig[3] = 0;
	vmc_all.pos.z = vmc[0].epos.z;
	vmc_all.pos_n.z = -vmc[0].epos.z;
	vmc_all.spd_n.z = 0;
	vmc_all.param.cog_off_use[3] = 0;

	vmc[FL1].force[Xr] = vmc[FL2].force[Xr] = vmc[BL1].force[Xr] = vmc[BL2].force[Xr] = 0;
	vmc[FL1].force[Yr] = vmc[FL2].force[Yr] = vmc[BL1].force[Yr] = vmc[BL2].force[Yr] = 0;
	vmc[FL1].force[Zr] = vmc[FL2].force[Zr] = vmc[BL1].force[Zr] = vmc[BL2].force[Zr] = 0;

	for (i = 0; i < 4; i++) {
		vmc[i].epos_td_hn.x = vmc[i].epos_lf_hn.x = 0;
		vmc[i].epos_td_hn.y = vmc[i].epos_lf_hn.y = 0;
		vmc[i].epos_td_hn.z = vmc[i].epos_lf_hn.z = 0;
		vmc[i].odom_st.x = vmc[i].odom_st.y = 0;
		vmc[i].param.trig_state = 0;
	}

	vmc_all.odom_st.x = vmc_all.odom_st.y = 0;
	vmc_all.cog_pos_n.x = vmc_all.cog_pos_n.y = 0;
}

/* 足端触地检测 */
void touchdown_check(robotTypeDef* rob, float dt)//zhaodi 三次采样着地判断
{
	float delta_pos = { 0 };
	static float z_pos_reg[4] = { 0 };
	float w_z = 1;
	for (uint8_t i = 0; i < 4; i++)
	{

		switch (rob->Leg[i].trig_state)//来自vmc摆动状态  着地判断参数
		{
		case 99:case 0://ST
			if (vmc_all.gait_mode == TROT)
				rob->Leg[i].touch_z_param = pos_force_p.touch_z_param_trot_st;
			else
				rob->Leg[i].touch_z_param = pos_force_p.touch_z_param_st;
			break;
		case 1://LF 
			rob->Leg[i].touch_z_param = pos_force_p.touch_z_param_st;
			break;
		case 2://SW 
			w_z = limitw(fabs(MIN_Z - robotwb.Leg[i].epos_h.z) / fabs(MIN_Z - MAX_Z) + 0.5, 0.35, 1);
			rob->Leg[i].touch_z_param = pos_force_p.touch_z_param_sw;
			//printf("ss\n");
			break;
		case 3://Td 
			w_z = limitw(fabs(MIN_Z - robotwb.Leg[i].epos_h.z) / fabs(MIN_Z - MAX_Z) + 0.5, 0.35, 1);
			rob->Leg[i].touch_z_param = pos_force_p.touch_z_param_td;
			//printf("%f\n", pos_force_p.touch_z_param_td.st_td);
			break;
		case 22://SW 
			w_z = 1;
			rob->Leg[i].touch_z_param = pos_force_p.touch_z_param_bound_st;
			//printf("ss\n");
			break;
		default:
			rob->Leg[i].touch_z_param = pos_force_p.touch_z_param_st;
			break;
		}
		//
		//Touch
		delta_pos = (rob->Leg[i].epos_b.z - z_pos_reg[i]) / dt;
		z_pos_reg[i] = rob->Leg[i].epos_b.z;
		if (rob->Leg[i].is_touch_est == 0) {
			if (rob->Leg[i].force_est_n_output.z > rob->Leg[i].touch_z_param.st_td / w_z
				&& robotwb.Leg[i].force_est_n_length > rob->Leg[i].touch_z_param.st_td / w_z * 0.7
				&& fabs(delta_pos) < rob->Leg[i].touch_z_param.check_spd
				) {
				rob->Leg[i].touch_cnto[0]++;
			}
			else
				rob->Leg[i].touch_cnto[1] = rob->Leg[i].touch_cnto[0] = 0;
#if RUN_WEBOTS
			if (i == 0 && rob->Leg[i].trig_state == 3 && 0) {
				printf("TD int Leg=%d f=%f %f c=%f %f %d\n",
					i,
					robotwb.Leg[i].force_est_n_output.z,
					robotwb.Leg[i].force_est_n_length,
					robotwb.Leg[i].touch_z_param.st_td,
					delta_pos,
					rob->Leg[i].touch_cnto[0]);
			}
#endif
			if (rob->Leg[i].touch_cnto[0] >= rob->Leg[i].touch_z_param.check_td) {
				rob->Leg[i].is_touch_est = 1;
				rob->Leg[i].touch_cnto[1] = rob->Leg[i].touch_cnto[0] = 0;
			}
		}
		else {
			//Lift
			if (rob->Leg[i].force_est_n_output.z < rob->Leg[i].touch_z_param.st_lf)
				rob->Leg[i].touch_cnto[1]++;
			else
				rob->Leg[i].touch_cnto[1] = rob->Leg[i].touch_cnto[0] = 0;

			if (rob->Leg[i].touch_cnto[1] >= rob->Leg[i].touch_z_param.check_lf) {
				rob->Leg[i].is_touch_est = 0;
				rob->Leg[i].touch_cnto[1] = rob->Leg[i].touch_cnto[0] = 0;
			}
		}
		//rob->Leg[i].is_touch_est = wb_touch_sensor_get_value(foot[i]);
#if RUN_WEBOTS&&!RUN_PI
		rob->Leg[i].is_touch_est = wb_touch_sensor_get_value(foot[i]);
#endif
	}

	static float timer = 0;
	timer += 0.005;
	three_leg_sel = 0;
	if (timer > 8)
	{
		//robotwb.Leg[three_leg_sel].is_ground = robotwb.Leg[three_leg_sel].is_touch_est=0;
		//three_leg = 1;
	}
}


void estimate_GRF(float dt)
{
	char i;
	//------------------------------------估计足底力-----------------------------
	float temp[2] = { 0 };
	for (i = 0; i < 4; i++) {
		//使用真实测量反馈IQ
#if FORCE_FB_USE_REAL&&(!RUN_WEBOTS||RUN_PI)
		temp[0] = -(robotwb.Leg[i].jacobi_inv[0] * (robotwb.Leg[i].taom[0] - robotwb.Leg[i].tao_bias[0]) + (robotwb.Leg[i].jacobi_inv[2] * (robotwb.Leg[i].taom[1] - robotwb.Leg[i].tao_bias[1])));
		temp[1] = -(robotwb.Leg[i].jacobi_inv[1] * (robotwb.Leg[i].taom[0] - robotwb.Leg[i].tao_bias[0]) + (robotwb.Leg[i].jacobi_inv[3] * (robotwb.Leg[i].taom[1] - robotwb.Leg[i].tao_bias[1])));
#else
	//直接使用力矩命令
		temp[0] = -(robotwb.Leg[i].jacobi_inv[0] * (robotwb.Leg[i].taom_output[0] - 0 * robotwb.Leg[i].taom[0] - 0 * robotwb.Leg[i].tao_bias[0])
			+ (robotwb.Leg[i].jacobi_inv[2] * (robotwb.Leg[i].taom_output[1] - 0 * robotwb.Leg[i].taom[1] - 0 * robotwb.Leg[i].tao_bias[1])));
		temp[1] = -(robotwb.Leg[i].jacobi_inv[1] * (robotwb.Leg[i].taom_output[0] - 0 * robotwb.Leg[i].taom[0] - 0 * robotwb.Leg[i].tao_bias[0])
			+ (robotwb.Leg[i].jacobi_inv[3] * (robotwb.Leg[i].taom_output[1] - 0 * robotwb.Leg[i].taom[1] - 0 * robotwb.Leg[i].tao_bias[1])));
#endif
		if (isnan(robotwb.Leg[i].jacobi_inv[0]))
			robotwb.Leg[i].jacobi_inv[0] = 0;
		if (isnan(robotwb.Leg[i].jacobi_inv[1]))
			robotwb.Leg[i].jacobi_inv[1] = 0;

		robotwb.Leg[i].force_est_h.x = temp[0];
		robotwb.Leg[i].force_est_h.z = temp[1];
		//转换全局
		if (isnan(robotwb.Leg[i].force_est_h.x))
			robotwb.Leg[i].force_est_h.x = 0;
		if (isnan(robotwb.Leg[i].force_est_h.y))
			robotwb.Leg[i].force_est_h.y = 0;
		if (isnan(robotwb.Leg[i].force_est_h.z))
			robotwb.Leg[i].force_est_h.z = 0;
		converV_b_to_nw(robotwb.Leg[i].force_est_h, &robotwb.Leg[i].force_est_n);
#if F_EST_WITH_ROLL
		robotwb.Leg[i].force_est_n.z = robotwb.Leg[i].force_est_n.z * cosd(LIMIT(robotwb.now_att.roll, -10, 10));
#endif	

		//滤波
		if (isnan(robotwb.Leg[i].force_est_n.x))
			robotwb.Leg[i].force_est_n.x = 0;
		if (isnan(robotwb.Leg[i].force_est_n.y))
			robotwb.Leg[i].force_est_n.y = 0;
		if (isnan(robotwb.Leg[i].force_est_n.z))
			robotwb.Leg[i].force_est_n.z = 0;
		//printf("1%f %f %f\n", robotwb.Leg[i].force_est_n.x, robotwb.Leg[i].force_est_n.z,dt);
#if 1
		DigitalLPF(robotwb.Leg[i].force_est_h.x, &robotwb.Leg[i].force_est_h_output.x, FLT_GRF, dt);
		DigitalLPF(robotwb.Leg[i].force_est_h.z, &robotwb.Leg[i].force_est_h_output.z, FLT_GRF, dt);
		DigitalLPF(robotwb.Leg[i].force_est_n.x, &robotwb.Leg[i].force_est_n_output.x, FLT_GRF, dt);
		DigitalLPF(robotwb.Leg[i].force_est_n.z, &robotwb.Leg[i].force_est_n_output.z, FLT_GRF, dt);
#else
		robotwb.Leg[i].force_est_n_output = robotwb.Leg[i].force_est_n;
#endif
		//printf("2%f %f\n", robotwb.Leg[i].force_est_n_output.x, robotwb.Leg[i].force_est_n_output.z);
		//计算Jerk
		robotwb.Leg[i].dforce_est_n_output.x = (robotwb.Leg[i].force_est_n_output.x - robotwb.Leg[i].force_est_n_output_reg.x) / dt;
		robotwb.Leg[i].dforce_est_n_output.y = 0;
		robotwb.Leg[i].dforce_est_n_output.z = (robotwb.Leg[i].force_est_n_output.z - robotwb.Leg[i].force_est_n_output_reg.z) / dt;
		robotwb.Leg[i].force_est_n_length = sqrtf(powf(robotwb.Leg[i].force_est_n_output.x, 2) + powf(robotwb.Leg[i].force_est_n_output.z, 2));

		robotwb.Leg[i].force_est_n_output_reg = robotwb.Leg[i].force_est_n_output;

		robotwb.Leg[i].dforce_est_h_output.x = (robotwb.Leg[i].force_est_h_output.x - robotwb.Leg[i].force_est_h_output_reg.x) / dt;
		robotwb.Leg[i].dforce_est_h_output.y = 0;
		robotwb.Leg[i].dforce_est_h_output.z = (robotwb.Leg[i].force_est_h_output.z - robotwb.Leg[i].force_est_h_output_reg.z) / dt;
		robotwb.Leg[i].force_est_h_length = sqrtf(powf(robotwb.Leg[i].force_est_h_output.x, 2) + powf(robotwb.Leg[i].force_est_h_output.z, 2));

		robotwb.Leg[i].force_est_h_output_reg = robotwb.Leg[i].force_est_h_output;

	}
	//质量估计
	robotwb.mess_est = 0;
	if (robotwb.ground_num >= 3 && vmc_all.gait_mode == STAND_IMU) {
		for (i = 0; i < 4; i++)
			robotwb.mess_est += robotwb.Leg[i].dforce_est_n_output.z / gw * robotwb.Leg[i].is_ground;
		robotwb.mess_est = robotwb.mess_est / (robotwb.ground_num + 0.000001);
	}
}

void estimate_GRF1(float dt)
{
	char i;
	//------------------------------------估计足底力-----------------------------
	float temp[2];
	for (i = 0; i < 4; i++) {
		//使用真实测量反馈IQ
#if FORCE_FB_USE_REAL
		temp[0] = -(robotwb.Leg[i].jacobi_inv[0] * robotwb.Leg[i].taom[0] + (robotwb.Leg[i].jacobi_inv[2] * robotwb.Leg[i].taom[1]));
		temp[1] = -(robotwb.Leg[i].jacobi_inv[1] * robotwb.Leg[i].taom[0] + (robotwb.Leg[i].jacobi_inv[3] * robotwb.Leg[i].taom[1]));
#if F_EST_WITH_ROLL
		robotwb.Leg[i].force_est_h.x = temp[0] * cosd(LIMIT(robotwb.now_att.roll, -10, 10));
		robotwb.Leg[i].force_est_h.z = temp[1] * cosd(LIMIT(robotwb.now_att.roll, -10, 10));
#else
		robotwb.Leg[i].force_est_h.x = temp[0];
		robotwb.Leg[i].force_est_h.z = temp[1];
#endif
		//转换全局
		converV_b_to_nw(robotwb.Leg[i].force_est_h, &robotwb.Leg[i].force_est_n);
		//滤波
#if 1
		DigitalLPF(robotwb.Leg[i].force_est_h.x, &robotwb.Leg[i].force_est_h_output.x, FLT_GRF, dt);
		DigitalLPF(robotwb.Leg[i].force_est_h.z, &robotwb.Leg[i].force_est_h_output.z, FLT_GRF, dt);
		DigitalLPF(robotwb.Leg[i].force_est_n.x, &robotwb.Leg[i].force_est_n_output.x, FLT_GRF, dt);
		DigitalLPF(robotwb.Leg[i].force_est_n.z, &robotwb.Leg[i].force_est_n_output.z, FLT_GRF, dt);
#else
		robotwb.Leg[i].force_est_n_output = robotwb.Leg[i].force_est_n;
#endif
		//计算Jerk
		if (fabs(robotwb.Leg[i].force_est_n_output.x - robotwb.Leg[i].force_est_n_output_reg.x) < 30)//WS
			robotwb.Leg[i].dforce_est_n_output.x = (robotwb.Leg[i].force_est_n_output.x - robotwb.Leg[i].force_est_n_output_reg.x) / dt;

		if (fabs(robotwb.Leg[i].force_est_n_output.z - robotwb.Leg[i].force_est_n_output_reg.z) < 30)//BUG!!
			robotwb.Leg[i].dforce_est_n_output.z = (robotwb.Leg[i].force_est_n_output.z - robotwb.Leg[i].force_est_n_output_reg.z) / dt;

		if (fabs(robotwb.Leg[i].force_est_h_output.x - robotwb.Leg[i].force_est_h_output_reg.x) < 30)//WS
			robotwb.Leg[i].dforce_est_h_output.x = (robotwb.Leg[i].force_est_h_output.x - robotwb.Leg[i].force_est_h_output_reg.x) / dt;

		if (fabs(robotwb.Leg[i].force_est_h_output.z - robotwb.Leg[i].force_est_h_output_reg.z) < 30)//BUG!!
			robotwb.Leg[i].dforce_est_h_output.z = (robotwb.Leg[i].force_est_h_output.z - robotwb.Leg[i].force_est_h_output_reg.z) / dt;

		robotwb.Leg[i].force_est_n_length = sqrtf(powf(robotwb.Leg[i].force_est_n_output.x, 2) + powf(robotwb.Leg[i].force_est_n_output.z, 2));

		robotwb.Leg[i].force_est_n_output_reg = robotwb.Leg[i].force_est_n_output;
		robotwb.Leg[i].force_est_h_output_reg = robotwb.Leg[i].force_est_h_output;
#else
	//直接使用力矩命令
		temp[0] = -(robotwb.Leg[i].jacobi_inv[0] * (robotwb.Leg[i].taom_output[0] - 0 * robotwb.Leg[i].taom[0]) + (robotwb.Leg[i].jacobi_inv[2] * (robotwb.Leg[i].taom_output[1] - 0 * robotwb.Leg[i].taom[1])));
		temp[1] = -(robotwb.Leg[i].jacobi_inv[1] * (robotwb.Leg[i].taom_output[0] - 0 * robotwb.Leg[i].taom[0]) + (robotwb.Leg[i].jacobi_inv[3] * (robotwb.Leg[i].taom_output[1] - 0 * robotwb.Leg[i].taom[1])));

		robotwb.Leg[i].force_est_h_output.x = temp[0];
		robotwb.Leg[i].force_est_h_output.z = temp[1];

#if F_EST_WITH_ROLL
		robotwb.Leg[i].force_est_h.x = temp[0] * cosd(LIMIT(robotwb.now_att.roll, -10, 10));
		robotwb.Leg[i].force_est_h.z = temp[1] * cosd(LIMIT(robotwb.now_att.roll, -10, 10));
#else
		robotwb.Leg[i].force_est_h.x = temp[0];
		robotwb.Leg[i].force_est_h.z = temp[1];
#endif
		//转换全局
		converV_b_to_nw(robotwb.Leg[i].force_est_h, &robotwb.Leg[i].force_est_n);
		//滤波
#if 1
		DigitalLPF(robotwb.Leg[i].force_est_n.x, &robotwb.Leg[i].force_est_n_output.x, FLT_GRF, dt);
		DigitalLPF(robotwb.Leg[i].force_est_n.z, &robotwb.Leg[i].force_est_n_output.z, FLT_GRF, dt);
#else
		robotwb.Leg[i].force_est_n_output = robotwb.Leg[i].force_est_n;
#endif
		//计算Jerk
		if (fabs(robotwb.Leg[i].force_est_n_output.x - robotwb.Leg[i].force_est_n_output_reg.x) < 30)//WS
			robotwb.Leg[i].dforce_est_n_output.x = (robotwb.Leg[i].force_est_n_output.x - robotwb.Leg[i].force_est_n_output_reg.x) / dt;
		if (fabs(robotwb.Leg[i].force_est_n_output.z - robotwb.Leg[i].force_est_n_output_reg.z) < 30)//BUG!!
			robotwb.Leg[i].dforce_est_n_output.z = (robotwb.Leg[i].force_est_n_output.z - robotwb.Leg[i].force_est_n_output_reg.z) / dt;

		robotwb.Leg[i].force_est_n_length = sqrtf(powf(robotwb.Leg[i].force_est_n_output.x, 2) + powf(robotwb.Leg[i].force_est_n_output.z, 2));

		robotwb.Leg[i].force_est_n_output_reg = robotwb.Leg[i].force_est_n_output;
#endif
	}
	//质量估计
	robotwb.mess_est = 0;
	if (robotwb.ground_num >= 3 && vmc_all.gait_mode == STAND_IMU) {
		for (i = 0; i < 4; i++)
			robotwb.mess_est += robotwb.Leg[i].dforce_est_n_output.z / gw * robotwb.Leg[i].is_ground;
		robotwb.mess_est = robotwb.mess_est / (robotwb.ground_num + 0.000001);
	}
}


void state_estimator(float dt)//guji
{
	static float st;
	static char init;
	static float time[5];
	float temp_odom_spd[3] = { 0 };
	float att_use[3], err[3], cog_off_hover, temp_end_dis;
	char i;
	if (!init)
	{
		TD4_init(&odom_td[Xr], 15, 15, 15, 15);
		TD4_init(&odom_td[Yr], 15, 15, 15, 15);
		TD4_init(&odom_td[Zr], 15, 15, 15, 15);
	}

	if (0) {
		vmc_all.gait_time[2] = vmc_all.gait_time[0]
			- dead(fabs(vmc_all.tar_spd.z), gait_change_param[0])*gait_change_param[1]
			- dead(fabs(vmc_all.tar_spd.x) / MAX_SPD, 0.5)*gait_change_param[4]
			- dead(fabs(vmc_all.tar_spd.y) / MAX_SPD / 2, 0.5)*gait_change_param[4];
		vmc_all.gait_time[2] = LIMIT(vmc_all.gait_time[2], gait_change_param[5], 99);

		DigitalLPF(vmc_all.gait_time[2], &vmc_all.gait_time[3], FLT_AUTO_T, dt);
	}
	else
		vmc_all.gait_time[3] = vmc_all.gait_time[2] = vmc_all.gait_time[1] = vmc_all.gait_time[0];

	vmc_all.delay_time[1] = vmc_all.delay_time[0];
	st = vmc_all.gait_time[1] * vmc_all.gait_alfa;
	vmc_all.stance_time_auto = st;

	vmc_all.ground_num = 0;//统计touch脚数量
	vmc_all.ground_num_touch = 0;

	for (i = 0; i < 4; i++) {
		vmc[i].tar_spd.x = vmc_all.param.tar_spd_use_rc.x;
		vmc[i].tar_spd.y = vmc_all.param.tar_spd_use_rc.y;

		vmc_all.ground[0][i] = vmc[i].ground;
		vmc_all.ground[1][i] = vmc[i].ground_s;
		if (vmc[i].ground)
			vmc_all.ground_num++;
		if (vmc[i].is_touch)
			vmc_all.ground_num_touch++;
	}

	att_use[PITr] = vmc_all.att_vm[PITr];
	att_use[ROLr] = vmc_all.att_ctrl[ROLr];

	err[PITr] = LIMIT(vmc_all.tar_att[PITr] + vmc_all.tar_att_bias[PITr] + vmc_all.tar_att_off[PITr] - att_use[PITr], -8, 8);
	err[ROLr] = LIMIT(vmc_all.tar_att[ROLr] + vmc_all.tar_att_bias[ROLr] + vmc_all.tar_att_off[ROLr] - att_use[ROLr], -8, 8);


	float temp[2] = { 0 };
	if ((vmc[0].ground + vmc[2].ground != 0) && (vmc[1].ground + vmc[3].ground != 0)) {//-----------PITr
		temp[0] = (vmc[0].epos_n.z*vmc[0].ground + vmc[2].epos_n.z*vmc[2].ground) / (vmc[0].ground + vmc[2].ground);
		temp[1] = (vmc[1].epos_n.z*vmc[1].ground + vmc[3].epos_n.z*vmc[3].ground) / (vmc[1].ground + vmc[3].ground);

		DigitalLPF(-fast_atan2(temp[0] - temp[1],
			fabs(vmc[0].epos_n.x*vmc[0].ground + vmc[2].epos_n.x*vmc[2].ground) / (vmc[0].ground + vmc[2].ground) +
			fabs(vmc[1].epos_n.x*vmc[1].ground + vmc[3].epos_n.x*vmc[3].ground) / (vmc[1].ground + vmc[3].ground)
		)*57.3, &vmc_all.att_vm[PITr],
			FLT_ATT_CTRL[0], dt);
		//body	
		temp[0] = (vmc[0].epos_b.z*vmc[0].ground + vmc[2].epos_b.z*vmc[2].ground) / (vmc[0].ground + vmc[2].ground);
		temp[1] = (vmc[1].epos_b.z*vmc[1].ground + vmc[3].epos_b.z*vmc[3].ground) / (vmc[1].ground + vmc[3].ground);

		DigitalLPF(-fast_atan2(temp[0] - temp[1],
			fabs(vmc[0].epos_b.x*vmc[0].ground + vmc[2].epos_b.x*vmc[2].ground) / (vmc[0].ground + vmc[2].ground) +
			fabs(vmc[1].epos_b.x*vmc[1].ground + vmc[3].epos_b.x*vmc[3].ground) / (vmc[1].ground + vmc[3].ground)
		)*57.3, &vmc_all.att_vm_b[PITr],
			FLT_ATT_CTRL[0], dt);


		temp[0] = (vmc[0].spd_n.z*vmc[0].ground + vmc[2].spd_n.z*vmc[2].ground) / (vmc[0].ground + vmc[2].ground);
		temp[1] = (vmc[1].spd_n.z*vmc[1].ground + vmc[3].spd_n.z*vmc[3].ground) / (vmc[1].ground + vmc[3].ground);
		DigitalLPF(-(temp[0] - temp[1]) / vmc_all.H*57.3*k_rate_vm[0], &vmc_all.att_rate_vm[PITr],
			FLT_ATT_CTRL[2], dt);
	}

	if ((vmc[0].ground + vmc[1].ground != 0) && (vmc[2].ground + vmc[3].ground != 0)) {//------------ROLr
		temp[0] = (vmc[0].epos_n.z*vmc[0].ground + vmc[1].epos_n.z*vmc[1].ground) / (vmc[0].ground + vmc[1].ground);
		temp[1] = (vmc[2].epos_n.z*vmc[2].ground + vmc[3].epos_n.z*vmc[3].ground) / (vmc[2].ground + vmc[3].ground);

		DigitalLPF(fast_atan2(temp[0] - temp[1],
			fabs(vmc[0].epos_n.y*vmc[0].ground + vmc[1].epos_n.y*vmc[1].ground) / (vmc[0].ground + vmc[1].ground) +
			fabs(vmc[2].epos_n.y*vmc[2].ground + vmc[3].epos_n.y*vmc[3].ground) / (vmc[2].ground + vmc[3].ground)
		)*57.3, &vmc_all.att_vm[ROLr],
			FLT_ATT_CTRL[0], dt);

		temp[0] = (vmc[0].spd_n.z*vmc[0].ground + vmc[1].spd_n.z*vmc[1].ground) / (vmc[0].ground + vmc[1].ground);
		temp[1] = (vmc[2].spd_n.z*vmc[2].ground + vmc[3].spd_n.z*vmc[3].ground) / (vmc[2].ground + vmc[3].ground);
		DigitalLPF((temp[0] - temp[1]) / vmc_all.W*57.3*k_rate_vm[0], &vmc_all.att_rate_vm[ROLr],
			FLT_ATT_CTRL[2], dt);
	}

	float spd_length = 0;
	float w_odom[4];
	float yaw_w_temp[3];
	float v_circle;
	float circle_r;
	char w_leg_cnt = 0;
	float temp_w_spd = 0;
	if (vmc_all.param.leg_dof == 3)//运动学航向估算
	{
		for (i = 0; i < 4; i++)
		{
			if (vmc[i].ground)
			{
				spd_length = sqrt(pow(vmc[i].spd_n.x, 2) + pow(vmc[i].spd_n.y, 2));
				yaw_w_temp[0] = fast_atan2(vmc[i].epos_n.x, vmc[i].epos_n.y)*57.3;
				yaw_w_temp[1] = fast_atan2(vmc[i].spd_n.x, vmc[i].spd_n.y)*57.3;
				yaw_w_temp[2] = (90 + yaw_w_temp[0]) - yaw_w_temp[1];
				v_circle = cosd(yaw_w_temp[2])*spd_length;
				circle_r = sqrt(pow(vmc[i].epos_n.x, 2) + pow(vmc[i].epos_n.y, 2));
				w_odom[w_leg_cnt++] = v_circle / (circle_r + 0.00001);
			}
		}
		for (i = 0; i < w_leg_cnt; i++)
			temp_w_spd += w_odom[i];
		temp_w_spd /= w_leg_cnt;
		temp_w_spd = LIMIT(temp_w_spd, -100, 100);
		DigitalLPF(temp_w_spd*RAD_TO_DEG, &att_rate_vm_yaw, FLT_ATT_CTRL[2], dt);
	}
	else {//2自由度
		DigitalLPF((vmc_all.param.encoder_spd[R] - vmc_all.param.encoder_spd[Ls]) / (vmc_all.W / 2)*RAD_TO_DEG*k_rate_vm[1],
			&vmc_all.att_rate_vm[YAWr], FLT_ATT_CTRL[2], dt);
		att_rate_vm_yaw = vmc_all.att_rate_vm[YAWr];
	}

	DigitalLPF(vmc_all.att[PITr] - k_att_trig_cal[PITr] * (vmc_all.tar_att[PITr] + vmc_all.tar_att_bias[PITr] * 1 + vmc_all.tar_att_off[PITr]), &vmc_all.att_trig[PITr], FLT_ATT_TRIG, dt);
	DigitalLPF(vmc_all.att[ROLr] - k_att_trig_cal[ROLr] * (vmc_all.tar_att[ROLr] + vmc_all.tar_att_bias[ROLr] * 1 + vmc_all.tar_att_off[ROLr]), &vmc_all.att_trig[ROLr], FLT_ATT_TRIG, dt);
	vmc_all.att_trig[YAWr] = vmc_all.att[YAWr];
	DigitalLPF(vmc_all.att_rate[PITr], &vmc_all.att_rate_trig[PITr], FLT_ATT_TRIG, dt);
	DigitalLPF(vmc_all.att_rate[ROLr], &vmc_all.att_rate_trig[ROLr], FLT_ATT_TRIG, dt);
	DigitalLPF(vmc_all.att_rate[YAWr], &vmc_all.att_rate_trig[YAWr], FLT_ATT_TRIG, dt);

	vmc_all.att_trig[PITr] = LIMIT(vmc_all.att_trig[PITr], -45, 45);
	vmc_all.att_trig[ROLr] = LIMIT(vmc_all.att_trig[ROLr], -45, 45);

	static float att_rate_flt[3];
	DigitalLPF(vmc_all.att[PITr], &vmc_all.att_ctrl[PITr], FLT_ATT_CTRL[0], dt);
	DigitalLPF(vmc_all.att[ROLr], &vmc_all.att_ctrl[ROLr], FLT_ATT_CTRL[0], dt);
	vmc_all.att_ctrl[YAWr] = vmc_all.att[YAWr];
	DigitalLPF(vmc_all.att_rate[PITr], &att_rate_flt[PITr], FLT_ATT_CTRL[1], dt);
	DigitalLPF(vmc_all.att_rate[ROLr], &att_rate_flt[ROLr], FLT_ATT_CTRL[1], dt);
	DigitalLPF(vmc_all.att_rate[YAWr], &att_rate_flt[YAWr], FLT_ATT_CTRL_Y, dt);
	//机械角速度融合
#if USE_MODEL_FOR_ATT_CAL
#if USE_ESO_OBSEVER
	ESO_2(&att_rate_eso[ROLr], att_rate_flt[ROLr], vmc_all.att_rate_vm[ROLr], dt);
	vmc_all.att_rate_ctrl[ROLr] = att_rate_eso[ROLr].z[0];
	ESO_2(&att_rate_eso[PITr], att_rate_flt[PITr], vmc_all.att_rate_vm[PITr], dt);
	vmc_all.att_rate_ctrl[PITr] = att_rate_eso[PITr].z[0];
	ESO_2(&att_rate_eso[YAWr], att_rate_flt[YAWr], vmc_all.att_rate_vm[YAWr], dt);
	vmc_all.att_rate_ctrl[YAWr] = att_rate_eso[YAWr].z[0];
	DigitalLPF(att_rate_eso[PITr].disturb, &att_rate_eso[PITr].disturb_u, FLT_ATT_CTRL_DIS, dt);
	DigitalLPF(att_rate_eso[ROLr].disturb, &att_rate_eso[ROLr].disturb_u, FLT_ATT_CTRL_DIS, dt);
	DigitalLPF(att_rate_eso[YAWr].disturb, &att_rate_eso[YAWr].disturb_u, FLT_ATT_CTRL_DIS, dt);
#else
	vmc_all.att_rate_ctrl[PITr] = w_r_vm[0] * att_rate_flt[PITr] + (1 - w_r_vm[0])*vmc_all.att_rate_vm[PITr];
	vmc_all.att_rate_ctrl[ROLr] = w_r_vm[0] * att_rate_flt[ROLr] + (1 - w_r_vm[0])*vmc_all.att_rate_vm[ROLr];
	vmc_all.att_rate_ctrl[YAWr] = w_r_vm[0] * att_rate_flt[YAWr] + (1 - w_r_vm[0])*vmc_all.att_rate_vm[YAWr];
#endif			
	//vmc_all.att_rate_ctrl[YAWr]=att_rate_flt[YAWr];
#else
	vmc_all.att_rate_ctrl[PITr] = att_rate_flt[PITr];
	vmc_all.att_rate_ctrl[ROLr] = att_rate_flt[ROLr];
	vmc_all.att_rate_ctrl[YAWr] = att_rate_flt[YAWr];
#endif


	//-----------------------------------------------------质心状态估计----------------------------------------
	static float timer_no_shock[4] = { 0 };
	for (i = 0; i < 4; i++) {
		if (!vmc[i].ground)
			vmc[i].ground_noshock = timer_no_shock[i] = 0;
		else {
			timer_no_shock[i] += dt;

			if (timer_no_shock[i] >= TD_NO_SHOCK_TIME)
				vmc[i].ground_noshock = 1;
		}
	}
	//着地质心位置估计
	if (vmc_all.gait_mode == TROT) {
		if ((vmc[0].ground_noshock&&vmc[3].ground_noshock &&
			(vmc[0].param.trig_state == 0 && vmc[3].param.trig_state == 0) &&
			!vmc[1].ground_noshock && !vmc[2].ground_noshock) ||
			(vmc[1].ground_noshock&&vmc[2].ground_noshock &&
			(vmc[1].param.trig_state == 0 && vmc[2].param.trig_state == 0) &&
				!vmc[0].ground_noshock && !vmc[3].ground_noshock)) {
			//Xr
			DigitalLPF(-(vmc[0].epos_n.x*vmc[0].ground_noshock + vmc[1].epos_n.x*vmc[1].ground_noshock +
				vmc[2].epos_n.x*vmc[2].ground_noshock + vmc[3].epos_n.x*vmc[3].ground_noshock) /
				LIMIT(vmc[0].ground_noshock + vmc[1].ground_noshock + vmc[2].ground_noshock + vmc[3].ground_noshock, 1, vmc_all.ground_num),
				&vmc_all.pos_n.x,
				FLT_POS_X, dt);

			vmc_all.pos_n.x = LIMIT(vmc_all.pos_n.x, MIN_X, MAX_X);
			//Zr
			DigitalLPF((vmc[0].epos.z*vmc[0].ground_noshock + vmc[1].epos.z*vmc[1].ground_noshock +
				vmc[2].epos.z*vmc[2].ground_noshock + vmc[3].epos.z*vmc[3].ground_noshock) /
				LIMIT(vmc[0].ground_noshock + vmc[1].ground_noshock + vmc[2].ground_noshock + vmc[3].ground_noshock, 1, vmc_all.ground_num),
				&vmc_all.pos.z,
				FLT_POS_Z, dt);
			vmc_all.pos.z = LIMIT(vmc_all.pos.z, MAX_Z*0.95, MIN_Z*1.05);

			DigitalLPF(-(vmc[0].epos_n.z*vmc[0].ground_noshock + vmc[1].epos_n.z*vmc[1].ground_noshock +
				vmc[2].epos_n.z*vmc[2].ground_noshock + vmc[3].epos_n.z*vmc[3].ground_noshock) /
				LIMIT(vmc[0].ground_noshock + vmc[1].ground_noshock + vmc[2].ground_noshock + vmc[3].ground_noshock, 1, vmc_all.ground_num),
				&vmc_all.pos_n.z,
				FLT_POS_Z, dt);

			vmc_all.pos_n.z = LIMIT(vmc_all.pos_n.z, -MIN_Z * 1.05, -MAX_Z * 0.95);
		}
	}
	else if (vmc_all.gait_mode == CLIMB|| vmc_all.gait_mode == BOUND  ) {//

		if (gait_bound.force_control_mode == 2&& vmc_all.gait_mode == BOUND) {//独立bound
			if (gait_bound.slip[F].ground || gait_bound.slip[B].ground)
			{
				DigitalLPF(-(gait_bound.slip[F].hip_nb_now.x*gait_bound.slip[F].ground + gait_bound.slip[B].hip_nb_now.x*gait_bound.slip[B].ground)
					/ (gait_bound.slip[F].ground + gait_bound.slip[B].ground),
					&vmc_all.pos_n.x,
					FLT_POS_X, dt);

				vmc_all.pos_n.x = LIMIT(vmc_all.pos_n.x, MIN_X, MAX_X);
				//Zr
				DigitalLPF(-(gait_bound.slip[F].hip_nb_now.z*gait_bound.slip[F].ground + gait_bound.slip[B].hip_nb_now.z*gait_bound.slip[B].ground)
					/ (gait_bound.slip[F].ground + gait_bound.slip[B].ground),
					&vmc_all.pos_n.z,
					FLT_POS_Z, dt);

				vmc_all.pos_n.z = LIMIT(vmc_all.pos_n.z, -MIN_Z * 1.05, -MAX_Z * 0.95);
			}
		}
		else {
			if ((vmc[0].ground + vmc[1].ground + vmc[2].ground + vmc[3].ground >= 3)
				|| (vmc[0].ground&&vmc[1].ground && !vmc[2].ground && !vmc[3].ground)
				|| (!vmc[0].ground && !vmc[1].ground&&vmc[2].ground&&vmc[3].ground)
				) {
				//Xr
				DigitalLPF(-(vmc[0].epos_n.x*vmc[0].ground + vmc[1].epos_n.x*vmc[1].ground +
					vmc[2].epos_n.x*vmc[2].ground + vmc[3].epos_n.x*vmc[3].ground) / vmc_all.ground_num,
					&vmc_all.pos_n.x,
					FLT_POS_X, dt);

				vmc_all.pos_n.x = LIMIT(vmc_all.pos_n.x, MIN_X, MAX_X);
				//Zr
				DigitalLPF((vmc[0].epos.z*vmc[0].ground + vmc[1].epos.z*vmc[1].ground +
					vmc[2].epos.z*vmc[2].ground + vmc[3].epos.z*vmc[3].ground) / vmc_all.ground_num,
					&vmc_all.pos.z,
					FLT_POS_Z, dt);
				vmc_all.pos.z = LIMIT(vmc_all.pos.z, MAX_Z*0.95, MIN_Z*1.05);

				DigitalLPF(-(vmc[0].epos_n.z*vmc[0].ground + vmc[1].epos_n.z*vmc[1].ground +
					vmc[2].epos_n.z*vmc[2].ground + vmc[3].epos_n.z*vmc[3].ground) / vmc_all.ground_num,
					&vmc_all.pos_n.z,
					FLT_POS_Z, dt);

				vmc_all.pos_n.z = LIMIT(vmc_all.pos_n.z, -MIN_Z * 1.05, -MAX_Z * 0.95);
			}

			if (vmc[0].ground && vmc[2].ground)
			{
				DigitalLPF(-(vmc[0].epos_n_b.x*vmc[0].ground + vmc[2].epos_n_b.x*vmc[2].ground) / 2,
					&vmc_all.pos_n_b[F].x,
					FLT_POS_X, dt);

				vmc_all.pos_n_b[F].x = LIMIT(vmc_all.pos_n_b[F].x, MIN_X, MAX_X);
				//Zr
				DigitalLPF(-(vmc[0].epos_n_b.z*vmc[0].ground + vmc[2].epos_n_b.z*vmc[2].ground) / 2,
					&vmc_all.pos_n_b[F].z,
					FLT_POS_Z, dt);

				vmc_all.pos_n_b[F].z = LIMIT(vmc_all.pos_n_b[F].z, -MIN_Z * 1.05, -MAX_Z * 0.95);
			}

			if (vmc[1].ground && vmc[3].ground)
			{
				DigitalLPF(-(vmc[1].epos_n_b.x*vmc[1].ground + vmc[3].epos_n_b.x*vmc[3].ground) / 2,
					&vmc_all.pos_n_b[B].x,
					FLT_POS_X, dt);

				vmc_all.pos_n_b[B].x = LIMIT(vmc_all.pos_n_b[B].x, MIN_X, MAX_X);
				//Zr
				DigitalLPF(-(vmc[1].epos_n_b.z*vmc[1].ground + vmc[3].epos_n_b.z*vmc[3].ground) / 2,
					&vmc_all.pos_n_b[B].z,
					FLT_POS_Z, dt);

				vmc_all.pos_n_b[B].z = LIMIT(vmc_all.pos_n_b[B].z, -MIN_Z * 1.05, -MAX_Z * 0.95);
			}
		}
	}
	else {//站立 Pronk
		if ((vmc[0].ground + vmc[1].ground + vmc[2].ground + vmc[3].ground >= 3)
			|| (vmc[0].ground&&vmc[1].ground && !vmc[2].ground && !vmc[3].ground)
			|| (!vmc[0].ground && !vmc[1].ground&&vmc[2].ground&&vmc[3].ground)
			) {
			//Xr
			DigitalLPF(-(vmc[0].epos_n.x*vmc[0].ground + vmc[1].epos_n.x*vmc[1].ground +
				vmc[2].epos_n.x*vmc[2].ground + vmc[3].epos_n.x*vmc[3].ground) / vmc_all.ground_num,
				&vmc_all.pos_n.x,
				FLT_POS_X, dt);

			vmc_all.pos_n.x = LIMIT(vmc_all.pos_n.x, MIN_X, MAX_X);
			//Zr
			DigitalLPF((vmc[0].epos.z*vmc[0].ground + vmc[1].epos.z*vmc[1].ground +
				vmc[2].epos.z*vmc[2].ground + vmc[3].epos.z*vmc[3].ground) / vmc_all.ground_num,
				&vmc_all.pos.z,
				FLT_POS_Z, dt);
			vmc_all.pos.z = LIMIT(vmc_all.pos.z, MAX_Z*0.95, MIN_Z*1.05);

			DigitalLPF(-(vmc[0].epos_n.z*vmc[0].ground + vmc[1].epos_n.z*vmc[1].ground +
				vmc[2].epos_n.z*vmc[2].ground + vmc[3].epos_n.z*vmc[3].ground) / vmc_all.ground_num,
				&vmc_all.pos_n.z,
				FLT_POS_Z, dt);

			vmc_all.pos_n.z = LIMIT(vmc_all.pos_n.z, -MIN_Z * 1.05, -MAX_Z * 0.95);

		}
	}

	//质心速度估计  SPEED
	if (vmc_all.gait_mode == TROT) {
		if ((vmc[0].ground_noshock&&vmc[3].ground_noshock &&
			(vmc[0].param.trig_state == 0 && vmc[3].param.trig_state == 0) &&//WS
			!vmc[1].ground_noshock && !vmc[2].ground_noshock) ||
			(vmc[1].ground_noshock&&vmc[2].ground_noshock &&
			(vmc[1].param.trig_state == 0 && vmc[2].param.trig_state == 0) &&
				!vmc[0].ground_noshock && !vmc[3].ground_noshock)) {
			temp[0] = (vmc[0].spd_n.x*vmc[0].ground_noshock*vmc[0].is_touch + vmc[1].spd_n.x*vmc[1].ground_noshock*vmc[1].is_touch +
				vmc[2].spd_n.x*vmc[2].ground_noshock*vmc[2].is_touch + vmc[3].spd_n.x*vmc[3].ground_noshock*vmc[3].is_touch) /
				LIMIT(vmc[0].is_touch + vmc[1].is_touch + vmc[2].is_touch + vmc[3].is_touch, 1, 2);

			vmc_all.body_spd[Xr] = LIMIT(temp[0], -2, 2);

			temp[0] = (vmc[0].spd_n.y*vmc[0].ground_noshock + vmc[1].spd_n.y*vmc[1].ground_noshock +
				vmc[2].spd_n.y*vmc[2].ground_noshock + vmc[3].spd_n.y*vmc[3].ground_noshock) / 2;

			vmc_all.body_spd[Yr] = -LIMIT(temp[0], -2, 2);
			vmc_all.spd_n.y = -vmc_all.body_spd[Yr];

			temp[0] = (vmc[0].spd_n.z*vmc[0].ground_noshock*vmc[0].is_touch + vmc[1].spd_n.z*vmc[1].ground_noshock*vmc[1].is_touch +
				vmc[2].spd_n.z*vmc[2].ground_noshock*vmc[2].is_touch + vmc[3].spd_n.z*vmc[3].ground_noshock*vmc[3].is_touch) /
				LIMIT(vmc[0].is_touch + vmc[1].is_touch + vmc[2].is_touch + vmc[3].is_touch, 1, 2);

			vmc_all.body_spd[Zr] = LIMIT(temp[0], -2, 2);			 		 
		}
	}
	else if (vmc_all.gait_mode == CLIMB|| vmc_all.gait_mode == BOUND) {//


		if (gait_bound.force_control_mode == 2 && vmc_all.gait_mode == BOUND) {//独立bound
			if (gait_bound.slip[F].ground || gait_bound.slip[B].ground)
			{
				temp[0] = (gait_bound.slip[F].dhip_n_now.x*gait_bound.slip[F].ground 
					+ gait_bound.slip[B].dhip_n_now.x*gait_bound.slip[B].ground)
					/ (gait_bound.slip[F].ground + gait_bound.slip[B].ground);

				vmc_all.body_spd[Xr] = LIMIT(temp[0], -2, 2);

				//Zr
				temp[0] = (gait_bound.slip[F].dhip_n_now.z*gait_bound.slip[F].ground 
					+ gait_bound.slip[B].dhip_n_now.z*gait_bound.slip[B].ground)
					/ (gait_bound.slip[F].ground + gait_bound.slip[B].ground);

				vmc_all.body_spd[Zr] = LIMIT(temp[0], -2, 2);
				float spd_gps = wb_gps_get_speed(GPS);
			}
		}
		else {
			if ((vmc[0].ground&&vmc[3].ground) || (vmc[1].ground&&vmc[2].ground) ||
				vmc_all.ground_num >= 3
				) {
				temp[0] = (vmc[0].spd_n.x*vmc[0].ground + vmc[1].spd_n.x*vmc[1].ground +
					vmc[2].spd_n.x*vmc[2].ground + vmc[3].spd_n.x*vmc[3].ground) / vmc_all.ground_num;

				vmc_all.body_spd[Xr] = LIMIT(temp[0], -2, 2);

				temp[0] = (vmc[0].spd_n.y*vmc[0].ground + vmc[1].spd_n.y*vmc[1].ground +
					vmc[2].spd_n.y*vmc[2].ground + vmc[3].spd_n.y*vmc[3].ground) / vmc_all.ground_num;

				vmc_all.body_spd[Yr] = -LIMIT(temp[0], -2, 2);
				vmc_all.spd_n.y = -vmc_all.body_spd[Yr];

				temp[0] = (vmc[0].spd_n.z*vmc[0].ground + vmc[1].spd_n.z*vmc[1].ground +
					vmc[2].spd_n.z*vmc[2].ground + vmc[3].spd_n.z*vmc[3].ground) / vmc_all.ground_num;

				vmc_all.body_spd[Zr] = LIMIT(temp[0], -2, 2);
			}

			if ((vmc[0].ground&&vmc[2].ground)) {
				temp[0] = (vmc[0].spd_n.x*vmc[0].ground +
					vmc[2].spd_n.x*vmc[2].ground) / (vmc[0].ground + vmc[2].ground);

				vmc_all.body_spd_b[F][Xr] = LIMIT(temp[0], -2, 2);

				temp[0] = (vmc[0].spd_n.z*vmc[0].ground +
					vmc[2].spd_n.z*vmc[2].ground) / (vmc[0].ground + vmc[2].ground);

				vmc_all.body_spd_b[F][Zr] = LIMIT(temp[0], -2, 2);
			}

			if ((vmc[1].ground&&vmc[3].ground)) {
				temp[0] = (vmc[1].spd_n.x*vmc[1].ground +
					vmc[3].spd_n.x*vmc[3].ground) / (vmc[1].ground + vmc[3].ground);

				vmc_all.body_spd_b[B][Xr] = LIMIT(temp[0], -2, 2);

				temp[0] = (vmc[1].spd_n.z*vmc[1].ground +
					vmc[3].spd_n.z*vmc[3].ground) / (vmc[1].ground + vmc[3].ground);

				vmc_all.body_spd_b[B][Zr] = LIMIT(temp[0], -2, 2);
			}
		}
	}
	else {//站立
		if ((vmc[0].ground&&vmc[3].ground) || (vmc[1].ground&&vmc[2].ground) ||
			vmc_all.ground_num >= 3
			) {
			temp[0] = (vmc[0].spd_n.x*vmc[0].ground + vmc[1].spd_n.x*vmc[1].ground +
				vmc[2].spd_n.x*vmc[2].ground + vmc[3].spd_n.x*vmc[3].ground) / vmc_all.ground_num;

			vmc_all.body_spd[Xr] = LIMIT(temp[0], -2, 2);

			temp[0] = (vmc[0].spd_n.y*vmc[0].ground + vmc[1].spd_n.y*vmc[1].ground +
				vmc[2].spd_n.y*vmc[2].ground + vmc[3].spd_n.y*vmc[3].ground) / vmc_all.ground_num;

			vmc_all.body_spd[Yr] = -LIMIT(temp[0], -2, 2);
			vmc_all.spd_n.y = -vmc_all.body_spd[Yr];

			temp[0] = (vmc[0].spd_n.z*vmc[0].ground + vmc[1].spd_n.z*vmc[1].ground +
				vmc[2].spd_n.z*vmc[2].ground + vmc[3].spd_n.z*vmc[3].ground) / vmc_all.ground_num;

			vmc_all.body_spd[Zr] = LIMIT(temp[0], -2, 2);
		}
	}
	DigitalLPF(vmc_all.body_spd[Xr], &vmc_all.spd_n.x, FLT_SPD_END_X_ODOM, dt);
	DigitalLPF(vmc_all.body_spd[Zr], &vmc_all.spd_n.z, FLT_SPD_END_Z, dt);

	DigitalLPF(vmc_all.body_spd_b[F][Xr], &vmc_all.spd_n_b[F].x, FLT_SPD_END_X_ODOM, dt);
	DigitalLPF(vmc_all.body_spd_b[F][Zr], &vmc_all.spd_n_b[F].z, FLT_SPD_END_Z, dt);
	DigitalLPF(vmc_all.body_spd_b[B][Xr], &vmc_all.spd_n_b[B].x, FLT_SPD_END_X_ODOM, dt);
	DigitalLPF(vmc_all.body_spd_b[B][Zr], &vmc_all.spd_n_b[B].z, FLT_SPD_END_Z, dt);

	DigitalLPF(vmc[0].spd_n.x*vmc[0].ground + vmc[1].spd_n.x*vmc[1].ground, &vmc_all.param.encoder_spd[R], FLT_ENCODER, dt);
	DigitalLPF(vmc[2].spd_n.x*vmc[2].ground + vmc[3].spd_n.x*vmc[3].ground, &vmc_all.param.encoder_spd[Ls], FLT_ENCODER, dt);

	vmc_all.body_spd[YAWrr] = vmc_all.att_ctrl[YAWr];

	if (vmc_all.param.tar_spd_use_rc.x > 0)
		cog_off_hover = vmc_all.cog_off[F];
	else
		cog_off_hover = vmc_all.cog_off[B];
	DigitalLPF(cog_off_hover, &vmc_all.param.cog_off_use[3], FLT_COG_OFF, dt);

	if (vmc[0].ground || vmc[2].ground)
		temp[0] = (vmc[0].epos_n.x*vmc[0].ground + vmc[2].epos_n.x*vmc[2].ground) / (vmc[0].ground + vmc[2].ground);
	if (vmc[1].ground || vmc[3].ground)
		temp[1] = (vmc[1].epos_n.x*vmc[1].ground + vmc[3].epos_n.x*vmc[3].ground) / (vmc[1].ground + vmc[3].ground);

	if ((vmc[0].ground || vmc[2].ground) && (vmc[1].ground || vmc[3].ground))
		vmc_all.pos_vm_leg_n.x = temp[0] / 2 + temp[1] / 2;
	//----------------------------------------------------------------------------------------	

	vmc[FL1].force[Xr] = vmc[FL2].force[Xr] = vmc[BL1].force[Xr] = vmc[BL2].force[Xr] = 0;
	vmc[FL1].force[Yr] = vmc[FL2].force[Yr] = vmc[BL1].force[Yr] = vmc[BL2].force[Yr] = 0;
	vmc[FL1].force[Zr] = vmc[FL2].force[Zr] = vmc[BL1].force[Zr] = vmc[BL2].force[Zr] = 0;

	//----------------------------------估计爬行WALK-------------
	END_POS ankle_pos_b[4], ankle_pos_n[4];
	for (i = 0; i < 4; i++) {
		ankle_pos_b[i].x = vmc[i].flag_fb*vmc_all.H / 2;
		ankle_pos_b[i].y = vmc[i].flag_rl*vmc_all.W / 2;
		ankle_pos_b[i].z = 0;

		vmc_all.ankle_pos_b[i].x = ankle_pos_b[i].x;
		vmc_all.ankle_pos_b[i].y = ankle_pos_b[i].y;
		vmc_all.ankle_pos_b[i].z = ankle_pos_b[i].z;

		float RTb_n_target[3][3];
		float att_test_use[3];
		att_test_use[PITr] = vmc_all.att_trig[PITr];
		att_test_use[ROLr] = vmc_all.att_trig[ROLr];
		att_test_use[YAWr] = vmc_all.att_trig[YAWr];

		RTb_n_target[0][0] = cosd(-att_test_use[PITr]);  RTb_n_target[0][1] = sind(-att_test_use[PITr])*sind(-att_test_use[ROLr]); RTb_n_target[0][2] = sind(-att_test_use[PITr])*cosd(-att_test_use[ROLr]);
		RTb_n_target[1][0] = 0;												   RTb_n_target[1][1] = cosd(-att_test_use[ROLr]);													 RTb_n_target[1][2] = -sind(-att_test_use[ROLr]);
		RTb_n_target[2][0] = -sind(-att_test_use[PITr]); RTb_n_target[2][1] = cosd(-att_test_use[PITr])*sind(-att_test_use[ROLr]); RTb_n_target[2][2] = cosd(-att_test_use[PITr])*cosd(-att_test_use[ROLr]);

		converV_b_to_n_RT(RTb_n_target, 0, ankle_pos_b[i].x, ankle_pos_b[i].y, ankle_pos_b[i].z,
			&ankle_pos_n[i].x, &ankle_pos_n[i].y, &ankle_pos_n[i].z);

		END_POS epos_n_no_yaw;
		vmc_all.ankle_pos_n[i].x = ankle_pos_n[i].x + vmc_all.cog_pos_n.x;
		vmc_all.ankle_pos_n[i].y = ankle_pos_n[i].y + vmc_all.cog_pos_n.y;
		vmc_all.ankle_pos_n[i].z = ankle_pos_n[i].z;
	}

	//-----------------------------------------------------------------------
	float spd_nn[2];
#if !ODOM_USE_1
	vmc_all.cog_spd_n.x = vmc_all.spd_n.x*cosd(vmc_all.ground_att[YAWr]) - vmc_all.spd_n.y*sind(vmc_all.ground_att[YAWr]);//??????????
	vmc_all.cog_spd_n.y = vmc_all.spd_n.x*sind(vmc_all.ground_att[YAWr]) + vmc_all.spd_n.y*cosd(vmc_all.ground_att[YAWr]);
	vmc_all.cog_spd_n.z = vmc_all.spd_n.z;
#endif

#if TEST_TROT_SW//摆动测试
	if (vmc_all.gait_mode == TROT) {
		vmc_all.cog_spd_n.x = 0;
		vmc_all.cog_spd_n.y = 0;
	}
#endif

#if ODOM_SPD_MODE&&!ODOM_USE_1//运动里程计
	vmc_all.cog_pos_n.x += my_deathzoom_2(vmc_all.cog_spd_n.x, 0.0001)*dt;
	vmc_all.cog_pos_n.y += my_deathzoom_2(vmc_all.cog_spd_n.y, 0.0001)*dt;
	vmc_all.cog_pos_n.z = vmc_all.pos_n.z;
#endif

#if TEST_TROT_SW
	vmc_all.cog_spd_n.x = 0;
	vmc_all.use_ground_sensor_new = 0;
#endif

	//-----------------------里程计足端位置-------------------
	END_POS epos_n_no_yaw;
	for (i = 0; i < 4; i++)
	{
		epos_n_no_yaw.x = vmc[i].epos_n.x;
		epos_n_no_yaw.y = vmc[i].epos_n.y;
		epos_n_no_yaw.z = vmc[i].epos_n.z;

		vmc[i].epos_nn.x = epos_n_no_yaw.x*cosd(vmc_all.ground_att[YAWr]) - epos_n_no_yaw.y*sind(vmc_all.ground_att[YAWr]) + vmc_all.cog_pos_n.x;//???????
		vmc[i].epos_nn.y = epos_n_no_yaw.x*sind(vmc_all.ground_att[YAWr]) + epos_n_no_yaw.y*cosd(vmc_all.ground_att[YAWr]) + vmc_all.cog_pos_n.y;
		vmc[i].epos_nn.z = epos_n_no_yaw.z;
	}
}

//地形估计
void estimate_ground_att(float dt) {
	// %     逆+（从箭头往里看）
	// %      z   y 逆+
	// %      |  /
	// %      | /
	// %      |/
	// %      o—————x 逆+
	// %1FL   2FR   2    0
	// %
	// %4BL   3BR   3    1
	static char state = 0;
	static float timer = 0;
	char i;
	int id_swap[4] = { 1,2,0,3 };
	double Pn_now[4][3] = { 0 };;
	double Pn_now_use[4][3] = { 0 };;
	static double Pn_last[4][3] = { 0 };
	static double Pn_td[4][3] = { 0 };
	static double ATT_G[3] = { 0,0,0 };
	static double ATT_G_OUT[3] = { 0,0,0 };
	double ATT_OFF[3] = { 0,0,0 };
	double flag_g = 0;
	char G_flag[4] = { 0 };
	static char G_flag_use[4] = { 0 };
	static char G_reg[4] = { 0 };
	static char G_reg_diag[2] = { 0 };
	static float D_G_ATT[2] = { 0 }, ATT_G_REG[2] = { 0 };
	float w1 = 0;
	timer += dt;

	Pn_now[0][Xr] = vmc[2].epos_n.y;
	Pn_now[0][Yr] = vmc[2].epos_n.x;
	Pn_now[0][Zr] = vmc[2].epos_n.z;
	G_flag[0] = vmc[2].ground_noshock;
	Pn_now[1][Xr] = vmc[0].epos_n.y;
	Pn_now[1][Yr] = vmc[0].epos_n.x;
	Pn_now[1][Zr] = vmc[0].epos_n.z;
	G_flag[1] = vmc[0].ground_noshock;
	Pn_now[2][Xr] = vmc[1].epos_n.y;
	Pn_now[2][Yr] = vmc[1].epos_n.x;
	Pn_now[2][Zr] = vmc[1].epos_n.z;
	G_flag[2] = vmc[1].ground_noshock;
	Pn_now[3][Xr] = vmc[3].epos_n.y;
	Pn_now[3][Yr] = vmc[3].epos_n.x;
	Pn_now[3][Zr] = vmc[3].epos_n.z;
	G_flag[3] = vmc[3].ground_noshock;

	switch (state)
	{
	case 0:
		for (i = 0; i < 4; i++) {//采样
			if (G_flag[i] == 1) {
				Pn_td[i][Xr] = Pn_now[i][Xr];
				Pn_td[i][Yr] = Pn_now[i][Yr];
				Pn_td[i][Zr] = Pn_now[i][Zr];
				G_flag_use[i] = 1;
			}
		}
		if (G_flag_use[0] && G_flag_use[1] && G_flag_use[2] && G_flag_use[3])//初始化
			state++;
		break;
	case 1:
		if (vmc_all.gait_mode == TROT || vmc_all.gait_mode == F_TROT)
		{
			if (G_reg_diag[0] == 0 && (G_flag[0] && G_flag[2])) {//对角着地
				Pn_td[0][Xr] = Pn_now[0][Xr];
				Pn_td[0][Yr] = Pn_now[0][Yr];
				Pn_td[0][Zr] = Pn_now[0][Zr];
				Pn_td[2][Xr] = Pn_now[2][Xr];
				Pn_td[2][Yr] = Pn_now[2][Yr];
				Pn_td[2][Zr] = Pn_now[2][Zr];
			}
			if (G_reg_diag[1] == 0 && (G_flag[1] && G_flag[3])) {//对角着地
				Pn_td[1][Xr] = Pn_now[1][Xr];
				Pn_td[1][Yr] = Pn_now[1][Yr];
				Pn_td[1][Zr] = Pn_now[1][Zr];
				Pn_td[3][Xr] = Pn_now[3][Xr];
				Pn_td[3][Yr] = Pn_now[3][Yr];
				Pn_td[3][Zr] = Pn_now[3][Zr];
			}
			if (timer > G_EST_TIME) {
				ground_att_est_n(G_flag_use, Pn_td[0], Pn_td[1], Pn_td[2], Pn_td[3], ATT_OFF, ATT_G, &flag_g);
				D_G_ATT[0] = (ATT_G[0] - ATT_G_REG[0]) / dt;
				D_G_ATT[1] = (ATT_G[1] - ATT_G_REG[1]) / dt;
				ATT_G_REG[0] = ATT_G[0];
				ATT_G_REG[1] = ATT_G[1];
				timer = 0;
			}
			else
			{
				ATT_G_OUT[0] += D_G_ATT[0] * dt * dt;
				ATT_G_OUT[1] += D_G_ATT[1] * dt * dt;

				w1 = 0.96;
				ATT_G_OUT[0] = ATT_G_OUT[0] * w1 + (1 - w1)*ATT_G[0];
				ATT_G_OUT[1] = ATT_G_OUT[1] * w1 + (1 - w1)*ATT_G[1];
			}
		}
		else if (vmc_all.gait_mode == STAND_IMU || vmc_all.gait_mode == STAND_RC
			|| vmc_all.gait_mode == PRONK
			|| vmc_all.gait_mode == BOUND
			|| vmc_all.gait_mode == CLIMB)//站立
		{
			if (1) {//对角着地
				Pn_td[0][Xr] = Pn_now[0][Xr];
				Pn_td[0][Yr] = Pn_now[0][Yr];
				Pn_td[0][Zr] = Pn_now[0][Zr];
				Pn_td[2][Xr] = Pn_now[2][Xr];
				Pn_td[2][Yr] = Pn_now[2][Yr];
				Pn_td[2][Zr] = Pn_now[2][Zr];
			}
			if (1) {//对角着地
				Pn_td[1][Xr] = Pn_now[1][Xr];
				Pn_td[1][Yr] = Pn_now[1][Yr];
				Pn_td[1][Zr] = Pn_now[1][Zr];
				Pn_td[3][Xr] = Pn_now[3][Xr];
				Pn_td[3][Yr] = Pn_now[3][Yr];
				Pn_td[3][Zr] = Pn_now[3][Zr];
			}
			if (timer > G_EST_TIME) {
				ground_att_est_n(G_flag_use, Pn_td[0], Pn_td[1], Pn_td[2], Pn_td[3], ATT_OFF, ATT_G_OUT, &flag_g);
				ATT_G_REG[0] = ATT_G_OUT[0];
				ATT_G_REG[1] = ATT_G_OUT[1];
				D_G_ATT[0] = D_G_ATT[1] = 0;
				timer = 0;
			}
		}
		else
		{
			G_flag_use[0] = G_flag_use[1] = G_flag_use[2] = G_flag_use[3] = 0;
			state = 0;
		}
		break;
	}

	for (i = 0; i < 4; i++)
		G_reg[i] = G_flag[i];
	// %0FL   1FR   2    0
	// %
	// %3BL   2BR   3    1  	int id_swap[4] = { 1,2,0,3 };
	G_reg_diag[0] = G_flag[0] && G_flag[2];
	G_reg_diag[1] = G_flag[1] && G_flag[3];

	DigitalLPF(-dead(ATT_G_OUT[1], DEAD_G_ATT), &vmc_all.ground_att_est[PITr], FLT_GROUND_ATT_EST, dt);
	DigitalLPF(dead(ATT_G_OUT[0], DEAD_G_ATT), &vmc_all.ground_att_est[ROLr], FLT_GROUND_ATT_EST, dt);
}


