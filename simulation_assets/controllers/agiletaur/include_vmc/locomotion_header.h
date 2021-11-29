#ifndef __LOCOMOITON_H__
#define __LOCOMOITON_H__
#include "base_struct.h"
//----------------------------------------------VMC controller---------------------
void body_traj_planner(float dt);
void body_servo_control(float dt);
void force_dis_n(void);
void force_dis_n_side(void);
void reset_servo_interge(void);

//---------------------------------------------Force IMP controller----------------------
void force_control_and_dis_stand(float dt);
void force_control_and_dis_stand_release(float dt);
void force_control_and_dis_stand_local(float dt);
void force_control_and_dis_trot(float dt);
void force_control_and_dis_trot_release(float dt);
void force_control_and_dis_trot_local(float dt);
//---------------------------------------------state estimator-------------------------
typedef struct{
	END_POS acc_n_m;
	END_POS acc_n_f;
	END_POS acc_nn_f;
	
	END_POS odom_spd_n;
	END_POS odom_pos_n;

	END_POS odom_spd_nn;
	END_POS odom_pos_nn;
	
	END_POS odom_dspd_n;
	END_POS odom_dpos_n;
	
	END_POS odom_spd_n_r;
	END_POS odom_pos_n_r;
	float Kpv;
}_ODOM_1;
extern _ODOM_1 odom1;

typedef struct 
{
  int state_gait;

}Gait_Mode;

extern Gait_Mode gait_ww;
extern char sit_down_flag;

void reset_robot_statement(void);
void touchdown_check(robotTypeDef* rob, float dt);
void estimate_GRF(float dt);
void state_estimator(float dt);
void estimate_ground_att(float dt);

//---------------------------------------------hardware interface---------------------
void subscribe_imu_to_webot(robotTypeDef* rob,float dt);
void subscribe_webot_to_vmc(float dt);
void subscribe_webot_to_vmc1(float dt);
void subscribe_webot_to_vmc2(float dt);
void subscribe_webot_to_vmc3(float dt);
void subscribe_webot_to_vmc4(float dt);
void publish_vmc_to_webot(float dt);
void readAllMotorPos(robotTypeDef* rob,float dt);
void set_motor_q(int id);
void set_motor_t(int id);
void pos_control_pd(float dt);
#if RUN_WEBOTS
#define NO_RC 1
void updateJoy(void);
void recorder(float dt);
void record_thread(char sel, float dt);
void updateKey(void);
#endif
//---------------------------------------------leg planner--------------------------------
void cal_vir_leg_pos(VMC* in, END_POS *td_vir_n, END_POS *lf_vir_n);
float end_pos_y_correction(VMC* in, float ep_slip_n, float dt);
void trig_curve_global(VMC *in, float *x, float *y, float *z, float *vx, float *vy, float *vz, float dt);
void espd_to_qspd(VMC *in, END_POS spd_end, float* spd_q, float dt);
void reset_sw_torque_as_now(VMC *in, float torque_now[3]);
void trig_plan(char leg_sel_trig, float dt);
char trig_lift(char leg_sel_trig, float dt);
char trig_swing(char leg_sel_trig, float dt);
char trig_td(char leg_sel_trig, float dt);
void swing_spd_control(char leg_sel_trig, float dt);
void reset_tar_pos(char leg_sel_trig);
void trig_plan_online(char leg_sel_trig, float dt);

//---------------------------------------------locomotion SFM--------------------------
typedef struct{
	char sdk_connect;
	int sdk_loss_cnt;
	char sdk_mode;
	char gait_mode;
	char cmd_power;
	char trig_mode;
	float cmd_vx;
	float cmd_vy;
	float cmd_vz;
	float cmd_vyaw;
	float cmd_z;
	float cmd_pit;
	float cmd_rol;
	float cmd_yaw;
	int cmd_pwm[12];
	float cmd_angle[4][3];
	END_POS trig_end_cmd[4];
}_SDK;
extern _SDK sdk;

extern char stand_force_enable_flag[5];
void vmc_param_init(void);
char safe_check(float dt);
void locomotion_sfm(float dt);
char gait_switch(float dt);
//-------------------------------------Gati  APP-----------------------------
typedef struct
{
	int state;
	int jump_trig;
	float timer[3];
	float jump_x_dis;
	float jump_yaw;
	float jump_w;
	float jump_height;
	float jump_height_off;
	float jump_power;
	float jump_avg_F[3];
	float jump_avg_T[3];
	float st_height;
	float ch_height;
	float couch_height;
	float damp_height;
	float lf_time;
	float couch_time;
	float td_time;
	float fly_time, fly_time_real;
	float lf_spd[3];
	float td_spd[3];

	float kp[3];
	float kp_T[3];
}Gait_Pronk;

extern Gait_Pronk gait_pronk;


typedef struct
{
	int state;
	int jump_trig;
	int jump_phase;
	int jump_phase_other;
	int force_control_mode;
	int up_stair;
	float timer[3];
	float jump_spdx;
	float jump_yaw;
	float jump_w;
	float jump_height;
	float jump_height_off;
	float jump_power;
	float jump_avg_F[3];
	float jump_avg_T[3];
	float st_height;
	float ch_height;
	float couch_height;
	float damp_height;
	float lf_time;
	float couch_time;
	float td_time;
	float fly_time, fly_time_real;
	float lf_spd[3];
	float td_spd[3];

	float kp[3];
	float fp[3];
	float kp_T[3];
}Gait_Climb;

extern Gait_Climb gait_climb;


typedef struct
{
	int state_all;
	int first_bound;
	float T_sw;
	float k_time;
	float L;
	float vd;
	float exp_z;
	float c;
	float mess;
	float s_peak_Fz;
	float tauP;
}Gait_Bound_IMP_Param;


typedef struct
{
	int state;
	int phase;
	int other_phase;
	char touch,ground;
	char id_sw[2];
	END_POS hip_n_exp;
	END_POS hip_n_now;
	END_POS hip_nb_now;
	END_POS dhip_n_exp;
	END_POS dhip_n_now;
	float att_exp[3];
	float att_now_rate[3];
	float att_now[3];
	float pid_hip[3];
	float pid_v[3];
	float pid_P[3];
	float k_jump;
	float T,T_tirg;
	float t;
	float t_start;
	float s;
	float T_st_norm;
	float T_st_now, T_st_now_trig;
	float T_st_last;
	float T_sw;
	float T_air;
	float alfa_Fz;
	float alfa_tauP;
	float Fz_imp;
	float TauP_imp;
	float g_st;
	float Fx;
	float Fz;

	float time_fly_near;
}Gait_Bound_IMP;


typedef struct
{
	Gait_Bound_IMP_Param slip_param;
	Gait_Bound_IMP slip[2];
	Gait_Bound_IMP slip_all;
	int state;
	int jump_trig;
	int jump_phase;
	int jump_phase_other;
	int force_control_mode;
	int up_stair;
	float timer[3];
	float jump_spdx;
	float jump_yaw;
	float jump_w;
	float jump_height;
	float jump_height_off;
	float jump_power;
	float jump_avg_F[3];
	float jump_avg_T[3];
	float st_height;
	float ch_height;
	float couch_height;
	float damp_height;
	float lf_time;
	float couch_time;
	float td_time;
	float fly_time, fly_time_real;
	float lf_spd[3];
	float td_spd[3];

	float kp[3];
	float fp[3];
	float kp_T[3];

}Gait_Bound;

extern Gait_Bound gait_bound;

void  Gait_Stand_Active(void);
void  Gait_Stand_Update(float dt);

void  Gait_Trot_Active(void);
void  Gait_Trot_Update_v1(float dt);

void  Gait_Recovery_Active(void);
void  Gait_Recovery_Falling(float dt);
void  Gait_Recovery_Update(float dt);

void  Gait_Pronk_Active(void);
void  Gait_Pronk_Update(float dt);

void  Gait_Climb_Active(void);
void  Gait_Climb_Update(float dt);

void  Gait_Bound_Active(void);
void  Gait_Bound_Update(float dt);

char move_joint_to_pos(VMC * in, int joint, float tar_angle, float max_spd, float dt);
char move_joint_to_pos1(VMC * in, int joint, float tar_angle, float max_spd, float err_check, float dt);
void move_joint_with_spd(VMC * in, int joint, float tar_spd, float dt);

#endif 