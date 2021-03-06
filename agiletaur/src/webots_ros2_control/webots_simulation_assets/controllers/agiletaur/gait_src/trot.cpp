#include "gait_math.h"
#include "eso.h"
#include "locomotion_header.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define TEST11 1//全局摆动
float k_att_trig[2] = { 0.25,0.25 };
int mmy_key[6] = { 0 };
int iid_off[3] = { 0 };
static char ukey_state = 0;
static char block_lisence = 1;
static char version_check = 0;
char block_yaw = 0;//���κ������
char  out_flag = 0;
float min_st_rate = 0.7;
_LISENCE lisens_vmc;

void get_license(void)//��ȡϵͳ�汾
{}

int board_id_test[3] = { 44,16,55 };
int board_license_test[3];
char check_lisence(void)
{
	lisens_vmc.key_right = vmc_all.key_right = 1;
	ukey_state = 96;
	return ukey_state;
}


void  Gait_Trot_Active(void)
{
	char i = 0;
	vmc_all.param.robot_mode = M_TROT;
	vmc_all.gait_mode = TROT;
	vmc_all.tar_att[YAWr] = vmc_all.att_ctrl[YAWr];
	vmc_all.tar_pos.z = vmc_all.pos_n.z;
	vmc_all.trot_state = 0;
	printf("Trot Active!\n");
	for (i = 0; i < 4; i++) {
		vmc[i].tar_sita1 = vmc[i].sita1;
		vmc[i].tar_sita2 = vmc[i].sita2;
		vmc[i].tar_sita3 = vmc[i].sita3;
		vmc[i].ground = 1;
		vmc_all.param.trot_sw_end_flag[i] = 0;
		vmc_all.param.gait_sw_end_reg_n[i] = vmc[i].epos_n;
		reset_tar_pos(i);
	}
	if (vmc_all.ground_num >= 2 && stand_force_enable_flag[4] && USE_FPOS_CONTROL && !TEST_TROT_SW) {//使能力控
		stand_force_enable_flag[0] = 1;
		stand_force_enable_flag[1] = 1;
		stand_force_enable_flag[2] = 1;
		stand_force_enable_flag[3] = 1;
		stand_force_enable_flag[4] = 1;
	}
}

static float k_att_test[3] = { -60,0.5,1 };
static char en_yaw_test = 0;
static float off_y = 0.015;
int en_att_mask = 1;
float FLT_PIT_TEST = 10;
char block_yaw_vmc = 1;
float trot_gcheck_time[3] = { 0.05,0.05 };//着地时间判断与离地时间

void  Gait_Trot_Update_v1(float dt)
{
	char trig_state_flag[4] = { 0,0,0,0 };
	static char init[4], init_lisence = 0, state, cnt_time_change, switch_flag;
	static float time_delay, yaw_force;
	static float timer_task[10];
	char i, sel;
	float test_epos_b[3], test_epos_leg[3];
	static END_POS end_pos_n[4], end_pos_n_use[4];//ĩ�˹̶���ȫ������ϵ�µ�λ��
	static float cnt_mode_can_switch = 0, mode_en_switch_flag = 0;
	static float att_test_use[3];
	char leg_sel_trig = 0;
	static float en_force_timer[3] = { 0 };
	float att_off[2];
	static int id_sw[2] = { FL1,BL2 };
	static float phast_time = 0;

	if (timer_task[2] > 0.02) {
		body_traj_planner(timer_task[2]);
		timer_task[2] = 0;
	}
	if (1) {//vmc_all.power_state>=2&&(ukey_state==96||block_lisence)){
		if (vmc_all.gait_mode == TROT || vmc_all.gait_mode == F_TROT)//----------------------�Խǲ�̬
		{
		
#if USE_MIT_SCHE 
			phase_gait_schedual_new(&gait, dt);
			event_gait_schedual(&gait, dt);
#else
			switch (vmc_all.trot_state)
			{
			case 0:
				if (EN_SW) {

					if (cnt_time_change++ > 2 || 1) {//WS
						vmc_all.gait_time[1] = vmc_all.gait_time[3];
						vmc_all.stance_time = vmc_all.stance_time_auto;
						vmc_all.delay_time[2] = vmc_all.delay_time[1];
						cnt_time_change = 0;
					}

					if (switch_flag)
					{
						id_sw[0] = FL1; id_sw[1] = BL2;
					}
					else
					{
						id_sw[0] = FL2; id_sw[1] = BL1;
					}

					for (i = 0; i < 2; i++) {
						trig_plan(id_sw[i], dt);//内部			reset_tar_pos(leg_sel_trig);
						vmc[id_sw[i]].ground = 0;	//清除着地标志位
						stand_force_enable_flag[id_sw[i]] = 0;//关闭力控 进入摆动
						vmc[id_sw[i]].param.trig_state = 1;

					}
					phast_time = 0;
					vmc_all.trot_state++;
				}
				break;
			case 1://lift up
				phast_time += dt;
				trig_state_flag[id_sw[0]] = trig_lift(id_sw[0], dt);
				trig_state_flag[id_sw[1]] = trig_lift(id_sw[1], dt);

				if (trig_state_flag[id_sw[0]] && trig_state_flag[id_sw[1]]) {//同时达到？
					vmc[id_sw[0]].param.trig_state = vmc[id_sw[1]].param.trig_state = 2;//进入摆动
					trig_state_flag[id_sw[0]] = trig_swing(id_sw[0], dt);
					trig_state_flag[id_sw[1]] = trig_swing(id_sw[1], dt);
					vmc_all.trot_state++;
				}
				break;
			case 2://swing and td
				phast_time += dt;
				for (i = 0; i < 2; i++) {
					switch (vmc[id_sw[i]].param.trig_state)
					{
					case 2:
						trig_state_flag[id_sw[i]] = trig_swing(id_sw[i], dt);

						if (trig_state_flag[id_sw[i]] == 2) {//摆动着地 之前未添加在舵狗
							vmc[id_sw[i]].param.trig_state = 4;
						}
						else if (trig_state_flag[id_sw[i]] == 1) {//摆动结束-->TD
							vmc[id_sw[i]].param.trig_state = 3;
							trig_td(id_sw[i], dt);
						}
						break;
					case 3:
						trig_state_flag[id_sw[i]] = trig_td(id_sw[i], dt);
						if (trig_state_flag[id_sw[i]] >= 1) {//TD结束正常
							vmc[id_sw[i]].param.trig_state = 4;
						}
						break;
					}
				}//end for

#if GROUND_AFTER_TRIG			
				if ((vmc[id_sw[0]].param.trig_state >= 4) && (vmc[id_sw[1]].param.trig_state >= 4)
#if FIX_ST_TIME
					&&phast_time > vmc_all.stance_time*MIN_ST_TIME_RATE
#endif
					)//对角腿同时着地
#else
				if ((vmc[id_sw[0]].ground&&vmc[id_sw[1]].ground)
#if FIX_ST_TIME
					&&phast_time > vmc_all.stance_time*MIN_ST_TIME_RATE
#endif
					)//对角腿同时着地
#endif
				{
					for (i = 0; i < 2; i++) {
						vmc[id_sw[i]].ground = 1;//切换着地符号  非Ground下Load力拉至地面 用于状态估计
						vmc[id_sw[i]].param.trig_state = 0;//复位摆动标志							
						stand_force_enable_flag[id_sw[i]] = 1;//启动力控
						//reset_tar_pos(id_sw[i]);
					}
					//着地切换							
					if (vmc_all.param.stand_trot_switch_flag == 1) {
						vmc_all.param.stand_switch_cnt[0]++;
						vmc_all.param.trot_sw_end_flag[id_sw[0]] = vmc_all.param.trot_sw_end_flag[id_sw[1]] = 1;
						if (vmc_all.param.stand_switch_cnt[0] >= 1)
						{
							vmc_all.param.stand_switch_cnt[0] = 0;
							vmc_all.param.stand_switch_flag[1] = 0;
							vmc_all.param.stand_trot_switch_flag = 0;
						}
					}

					if (vmc_all.param.stand_trot_switch_flag == 2) {
						vmc_all.param.stand_switch_cnt[1]++;
						switch (vmc_all.param.stand_switch_cnt[1])
						{
						case 1:
							vmc_all.param.stand_switch_flag[1] = 1;
							break;
						case 3:
							vmc_all.param.stand_switch_cnt[1] = 0;
							vmc_all.param.stand_trot_switch_flag = 0;
							break;
						}
					}
					//重新摆动
					if (vmc_all.delay_time[2] > 0) {
						time_delay = 0;
						vmc_all.trot_state++;
					}
					else
					{
						time_delay = 0; switch_flag = !switch_flag;
#if 1					
						if (cnt_time_change++ > 2 || 1) {//WS
							vmc_all.gait_time[1] = vmc_all.gait_time[3];	vmc_all.stance_time = vmc_all.stance_time_auto;		vmc_all.delay_time[2] = vmc_all.delay_time[1];			cnt_time_change = 0;
						}

						if (switch_flag)	//马上切换 NEW
						{
							id_sw[0] = FL1; id_sw[1] = BL2;

						}
						else
						{

							id_sw[0] = FL2; id_sw[1] = BL1;
						}

						for (i = 0; i < 2; i++) {
							trig_plan(id_sw[i], dt);//内部			reset_tar_pos(leg_sel_trig);
							vmc[id_sw[i]].ground = 0;	//清除着地标志位
							stand_force_enable_flag[id_sw[i]] = 0;//关闭力控 进入摆动
							vmc[id_sw[i]].param.trig_state = 1;

						}
						vmc_all.trot_state = 1;
#else
						vmc_all.trot_state = 0;
#endif
					}

				}
				break;
			case 3://等待
				time_delay += dt;
				if (time_delay > vmc_all.delay_time[2])
				{
					time_delay = 0; switch_flag = !switch_flag;

					if (cnt_time_change++ > 2 || 1) {//WS
						vmc_all.gait_time[1] = vmc_all.gait_time[3];	vmc_all.stance_time = vmc_all.stance_time_auto;		vmc_all.delay_time[2] = vmc_all.delay_time[1];			cnt_time_change = 0;
					}

					if (switch_flag)	//马上切换 NEW
					{
						id_sw[0] = FL1; id_sw[1] = BL2;
					}
					else
					{

						id_sw[0] = FL2; id_sw[1] = BL1;
					}

					for (i = 0; i < 2; i++) {
						trig_plan(id_sw[i], dt);//内部			reset_tar_pos(leg_sel_trig);
						vmc[id_sw[i]].ground = 0;	//清除着地标志位
						stand_force_enable_flag[id_sw[i]] = 0;//关闭力控 进入摆动
						vmc[id_sw[i]].param.trig_state = 1;

					}
					vmc_all.trot_state = 1;
				}
				break;
			}
#endif

			for (i = 0; i < 4; i++) {//输出摆动速度
				if (vmc[i].param.trig_state >= 1 && vmc[i].param.trig_state <= 3)// 
					swing_spd_control(i, dt);
			}
		}


	}	//end of lisence check

	for (i = 0; i < 10; i++)
		timer_task[i] += dt;
}
