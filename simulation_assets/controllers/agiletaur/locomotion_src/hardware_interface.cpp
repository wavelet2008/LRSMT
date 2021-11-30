#include "include.h"
#include "locomotion_header.h"
#include "can.h"
#include "gait_math.h"
#include "adrc.h"
#if RUN_WEBOTS
#include "wbInterface.h"
#endif
//��������������    ������
float FLT_ATT_RATE=0;//WS
float FLT_ATT_RT = 20;
void subscribe_imu_to_webot(robotTypeDef* rob,float dt)
{
#if !RUN_WEBOTS
	char i,j;
	robotwb.IMU_now.pitch=vmc_all.att[PITr];
	robotwb.IMU_now.roll=vmc_all.att[ROLr];
	robotwb.IMU_now.yaw=vmc_all.att[YAWr];
	
  for(i=0;i<3;i++)
  {
    for(j=0;j<3;j++){
    robotwb.Rb_n[i][j]=vmc_all.Rb_n[i][j];
    robotwb.Rn_b[i][j]=vmc_all.Rn_b[i][j];
    }
  }

	DigitalLPF_Double(vmc_all.att_rate[PITr], &robotwb.IMU_dot.pitch, FLT_ATT_RATE, dt);
	DigitalLPF_Double(vmc_all.att_rate[ROLr], &robotwb.IMU_dot.roll, FLT_ATT_RATE, dt);
	DigitalLPF_Double(vmc_all.att_rate[YAWr], &robotwb.IMU_dot.yaw, FLT_ATT_RATE, dt);

	robotwb.now_att=robotwb.IMU_now;
	robotwb.now_rate=robotwb.IMU_dot;
#else
	const double* data = wb_inertial_unit_get_roll_pitch_yaw(IMU);

	rob->IMU_now.roll = -deg(data[1]);
	rob->IMU_now.yaw = deg(data[2]);
	rob->IMU_now.pitch = deg(data[0]);

	rob->IMU_dot.roll = (rob->IMU_now.roll - rob->IMU_last.roll) / dt;
	rob->IMU_dot.yaw = (rob->IMU_now.yaw - rob->IMU_last.yaw) / dt;
	rob->IMU_dot.pitch = (rob->IMU_now.pitch - rob->IMU_last.pitch) / dt;

	rob->IMU_last.roll = rob->IMU_now.roll;
	rob->IMU_last.yaw = rob->IMU_now.yaw;
	rob->IMU_last.pitch = rob->IMU_now.pitch;

	rob->now_att = rob->IMU_now;
	rob->now_rate = rob->IMU_dot;

	vmc_all.att[PITr]= robotwb.IMU_now.pitch;
	vmc_all.att[ROLr]= robotwb.IMU_now.roll;
	vmc_all.att[YAWr]= robotwb.IMU_now.yaw;

	vmc_all.att_rate[PITr]=robotwb.IMU_dot.pitch;
	vmc_all.att_rate[ROLr]=robotwb.IMU_dot.roll;
	vmc_all.att_rate[YAWr]=robotwb.IMU_dot.yaw;

	static float att_rt_use[3] = {0};
	DigitalLPF(vmc_all.att[PITr], &att_rt_use[PITr], FLT_ATT_RT, dt);
	DigitalLPF(vmc_all.att[ROLr], &att_rt_use[ROLr], FLT_ATT_RT, dt);
	DigitalLPF(vmc_all.att[YAWr], &att_rt_use[YAWr], FLT_ATT_RT, dt);

	att_rt_use[YAWr] = 0;
	vmc_all.Rn_b[0][0] = cosd(-att_rt_use[PITr])*cosd(-att_rt_use[YAWr]);
	vmc_all.Rn_b[1][0] = -cosd(-att_rt_use[ROLr])*sind(-att_rt_use[YAWr]) + sind(-att_rt_use[PITr])*sind(-att_rt_use[ROLr])*cosd(-att_rt_use[YAWr]);
	vmc_all.Rn_b[2][0] = sind(-att_rt_use[ROLr])*sind(-att_rt_use[YAWr]) + cosd(-att_rt_use[ROLr])*sind(-att_rt_use[PITr])*cosd(-att_rt_use[YAWr]);

	vmc_all.Rn_b[0][1] = cosd(-att_rt_use[PITr])*sind(-att_rt_use[YAWr]);
	vmc_all.Rn_b[1][1] = cosd(-att_rt_use[ROLr])*cosd(-att_rt_use[YAWr]) + sind(-att_rt_use[ROLr])*sind(-att_rt_use[PITr])*sind(-att_rt_use[YAWr]);
	vmc_all.Rn_b[2][1] = -sind(-att_rt_use[ROLr])*cosd(-att_rt_use[YAWr]) + cosd(-att_rt_use[ROLr])*sind(-att_rt_use[PITr])*sind(-att_rt_use[YAWr]);

	vmc_all.Rn_b[0][2] = -sind(-att_rt_use[PITr]);
	vmc_all.Rn_b[1][2] = sind(-att_rt_use[ROLr])*cosd(-att_rt_use[PITr]);
	vmc_all.Rn_b[2][2] = cosd(-att_rt_use[ROLr])*cosd(-att_rt_use[PITr]);

	mat_trans(vmc_all.Rn_b, vmc_all.Rb_n);
	/*
	printf("---------------\n");
	printf("%f %f %f\n", vmc_all.Rn_b[0][0], vmc_all.Rn_b[0][1], vmc_all.Rn_b[0][2]);
	printf("%f %f %f\n", vmc_all.Rn_b[1][0], vmc_all.Rn_b[1][1], vmc_all.Rn_b[1][2]);
	printf("%f %f %f\n", vmc_all.Rn_b[2][0], vmc_all.Rn_b[2][1], vmc_all.Rn_b[2][2]);
	printf(" \n");
	printf("%f %f %f\n", vmc_all.Rb_n[0][0], vmc_all.Rb_n[0][1], vmc_all.Rb_n[0][2]);
	printf("%f %f %f\n", vmc_all.Rb_n[1][0], vmc_all.Rb_n[1][1], vmc_all.Rb_n[1][2]);
	printf("%f %f %f\n", vmc_all.Rb_n[2][0], vmc_all.Rb_n[2][1], vmc_all.Rb_n[2][2]);
	*/
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++) {
#if 1
			robotwb.Rb_n[i][j] = vmc_all.Rb_n[i][j];
			robotwb.Rn_b[i][j] = vmc_all.Rn_b[i][j];
#else
			robotwb.Rn_b[i][j] = vmc_all.Rb_n[i][j];
			robotwb.Rb_n[i][j] = vmc_all.Rn_b[i][j];
#endif
		}
	}
#endif
}

void subscribe_webot_to_vmc(float dt)//�����������ݸ�ֵ��VMC
{
  int i=0,j=0;
  for(i=0;i<4;i++)
  {
		leg_motor[i].t_to_i[0]=pos_force_p.t_to_i;//����Ť��ϵ��<<-----------------------------
		leg_motor[i].t_to_i[1]=leg_motor[i].t_to_i[0];
		
		#if Q_NOW_USE_SET
		vmc[i].sita1=vmc[i].tar_sita1;
		vmc[i].sita2=vmc[i].tar_sita2;
		#else
		if(vmc[i].param.q_now_use_tar){//ʹ��������Ϊ��ǰ����  �湷ģʽ
			vmc[i].sita1=vmc[i].tar_sita1;
			vmc[i].sita2=vmc[i].tar_sita2;		
		}else{
			vmc[i].sita1=robotwb.Leg[i].sita[0];
			vmc[i].sita2=robotwb.Leg[i].sita[1];
			vmc[i].dsita1=robotwb.Leg[i].sita_d[0];
			vmc[i].dsita2=robotwb.Leg[i].sita_d[1];	
			//printf("%f %f\n", vmc[i].sita1, vmc[i].sita2);
		}
		#endif

	vmc[i].delta_ht=vmc_robot_p.sw_deltah;
    robotwb.Leg[i].is_ground= vmc[i].ground;
    robotwb.Leg[i].trig_state=vmc[i].param.trig_state;
		
    #if !GROUND_USE_EST//ʹ���ŵش�����
    vmc[i].is_touch=robotwb.Leg[i].is_touch;
    #else
    vmc[i].is_touch=robotwb.Leg[i].is_touch_est;
    #endif
		robotwb.Leg[i].limit_tao[0]=pos_force_p.max_t;
		robotwb.Leg[i].limit_tao[1]=pos_force_p.max_t;  

		robotwb.Leg[i].f_pos_pid[Xr]=pos_force_p.f_pos_pid_st[Xr]; 
		robotwb.Leg[i].f_pos_pid[Zr]=pos_force_p.f_pos_pid_st[Zr]; 
		robotwb.Leg[i].f_pid[Xr]=pos_force_p.f_pid_st[Xr]; 
		robotwb.Leg[i].f_pid[Zr]=pos_force_p.f_pid_st[Zr]; 
		
		//�ؽ�PD����
		if(vmc_all.gait_mode==TROT||vmc_all.gait_mode==F_TROT)
		{
				if(stand_force_enable_flag[i]){
				robotwb.Leg[i].q_pid=pos_force_p.q_pid_st_trot; 	
				}	
				else
				{
				robotwb.Leg[i].q_pid=pos_force_p.q_pid_sw;//�ڶ�����
				}		
		}else{
				if(robotwb.Leg[i].is_ground){//֧�Ų���
				robotwb.Leg[i].q_pid=pos_force_p.q_pid_st_stance; 	
				}
				else
				{
				robotwb.Leg[i].q_pid=pos_force_p.q_pid_sw;//�ڶ�����
				}
		}
		leg_motor[i].max_t[0]=leg_motor[i].max_t[1]=leg_motor[i].max_t[2]=pos_force_p.max_t;
		leg_motor[i].max_i[0]=leg_motor[i].max_i[1]=leg_motor[i].max_i[2]=pos_force_p.max_i;
  }
	
 //���ν���������
 vmc_all.ground_att_cmd[PITr]=vmc_all.ground_att_est[PITr];

  robotwb.exp_att.pitch=vmc_all.tar_att[PITr]+LIMIT(vmc_all.ground_att_cmd[PITr]*(vmc_all.gait_mode>0)*EN_ATT_GROUND_CONTROL,-20,20);
  robotwb.exp_att.roll= vmc_all.tar_att[ROLr]+LIMIT(vmc_all.ground_att_cmd[ROLr]*(vmc_all.gait_mode>0)*EN_ATT_GROUND_CONTROL,-5,5);
}




void subscribe_webot_to_vmc1(float dt)//�����������ݸ�ֵ��VMC
{
	int i = 0, j = 0;
	for (i = 0; i < 4; i++)
	{

#if Q_NOW_USE_SET
		vmc[i].sita1 = vmc[i].tar_sita1;
		vmc[i].sita2 = vmc[i].tar_sita2;
#else
		if (vmc[i].param.q_now_use_tar) {//ʹ��������Ϊ��ǰ����  �湷ģʽ
			vmc[i].sita1 = vmc[i].tar_sita1;
			vmc[i].sita2 = vmc[i].tar_sita2;
		}
		else {
			vmc[i].sita1 = robotwb.Leg[i].sita[0];
			vmc[i].sita2 = robotwb.Leg[i].sita[1];
			vmc[i].dsita1 = robotwb.Leg[i].sita_d[0];
			vmc[i].dsita2 = robotwb.Leg[i].sita_d[1];
		}
#endif
	}
}

void subscribe_webot_to_vmc2(float dt)//�����������ݸ�ֵ��VMC
{
	int i = 0, j = 0;
	for (i = 0; i < 4; i++)
	{
		leg_motor[i].t_to_i[0] = pos_force_p.t_to_i;//����Ť��ϵ��<<-----------------------------
		leg_motor[i].t_to_i[1] = leg_motor[i].t_to_i[0];

		vmc[i].delta_ht = vmc_robot_p.sw_deltah;
		robotwb.Leg[i].is_ground = vmc[i].ground;
		robotwb.Leg[i].trig_state = vmc[i].param.trig_state;

		robotwb.Leg[i].limit_tao[0] = pos_force_p.max_t;
		robotwb.Leg[i].limit_tao[1] = pos_force_p.max_t;

		robotwb.Leg[i].f_pos_pid[Xr] = pos_force_p.f_pos_pid_st[Xr];
		robotwb.Leg[i].f_pos_pid[Zr] = pos_force_p.f_pos_pid_st[Zr];
		robotwb.Leg[i].f_pid[Xr] = pos_force_p.f_pid_st[Xr];
		robotwb.Leg[i].f_pid[Zr] = pos_force_p.f_pid_st[Zr];

		//�ؽ�PD����
		if (vmc_all.gait_mode == TROT || vmc_all.gait_mode == F_TROT)
		{
			if (robotwb.Leg[i].is_ground) {//stand_force_enable_flag[i]){
				robotwb.Leg[i].q_pid = pos_force_p.q_pid_st_trot;
			}
			else
			{
				robotwb.Leg[i].q_pid = pos_force_p.q_pid_sw;//�ڶ�����
			}
		}
		else {
			if (robotwb.Leg[i].is_ground) {//֧�Ų���
				robotwb.Leg[i].q_pid = pos_force_p.q_pid_st_stance;
			}
			else
			{
				robotwb.Leg[i].q_pid = pos_force_p.q_pid_sw;//�ڶ�����
			}
		}
		leg_motor[i].max_t[0] = leg_motor[i].max_t[1] = leg_motor[i].max_t[2] = pos_force_p.max_t;
		leg_motor[i].max_i[0] = leg_motor[i].max_i[1] = leg_motor[i].max_i[2] = pos_force_p.max_i;
	}
}


void subscribe_webot_to_vmc3(float dt)//�����������ݸ�ֵ��VMC
{
	int i = 0, j = 0;
	for (i = 0; i < 4; i++)
	{
#if !GROUND_USE_EST//ʹ���ŵش�����
		vmc[i].is_touch = robotwb.Leg[i].is_touch;
#else
		vmc[i].is_touch = robotwb.Leg[i].is_touch_est;
#endif
	}

	//���ν���������
	vmc_all.ground_att_cmd[PITr] = vmc_all.ground_att_est[PITr];

	robotwb.exp_att.pitch = vmc_all.tar_att[PITr] + LIMIT(vmc_all.ground_att_cmd[PITr] * (vmc_all.gait_mode > 0)*EN_ATT_GROUND_CONTROL, -20, 20);
	robotwb.exp_att.roll = vmc_all.tar_att[ROLr] + LIMIT(vmc_all.ground_att_cmd[ROLr] * (vmc_all.gait_mode > 0)*EN_ATT_GROUND_CONTROL, -5, 5);
}

void subscribe_webot_to_vmc4(float dt)//�����������ݸ�ֵ��VMC
{
	int i = 0, j = 0;

	for (i = 0; i < 4; i++)
	{
		vmc[i].delta_ht = vmc_robot_p.sw_deltah;
	}
	//���ν���������
	vmc_all.ground_att_cmd[PITr] = vmc_all.ground_att_est[PITr];
	//("p=%f\n", vmc_all.ground_att_cmd[PITr]);
	robotwb.exp_att.pitch = vmc_all.tar_att[PITr] + LIMIT(vmc_all.ground_att_cmd[PITr] * (vmc_all.gait_mode > 0)*EN_ATT_GROUND_CONTROL, -30, 30);
	robotwb.exp_att.roll = vmc_all.tar_att[ROLr] + LIMIT(vmc_all.ground_att_cmd[ROLr] * (vmc_all.gait_mode > 0)*EN_ATT_GROUND_CONTROL, -5, 5);
}


void publish_vmc_to_webot(float dt)
{
    int i=0;
    for(i=0;i<4;i++){
    float alfa=(robotwb.Leg[i].sita[1]-robotwb.Leg[i].sita[0])/2;
	  float beta=(robotwb.Leg[i].sita[1]+robotwb.Leg[i].sita[0])/2;
		robotwb.Leg[i].beta=beta;
		robotwb.Leg[i].alfa=alfa;

    robotwb.Leg[i].r=vmc[i].r;
    robotwb.Leg[i].sita_r=vmc[i].sita;
    robotwb.Leg[i].epos_h.x=vmc[i].epos.x;
	robotwb.Leg[i].epos_h.y=vmc[i].epos.y;
    robotwb.Leg[i].epos_h.z=vmc[i].epos.z;

	robotwb.Leg[i].epos_b.x=vmc[i].epos_b.x;
	robotwb.Leg[i].epos_b.y=vmc[i].epos_b.y;
	robotwb.Leg[i].epos_b.z=vmc[i].epos_b.z;

	robotwb.Leg[i].epos_n.x=vmc[i].epos_n.x;
	robotwb.Leg[i].epos_n.y=vmc[i].epos_n.y;
	robotwb.Leg[i].epos_n.z=vmc[i].epos_n.z;
		
    robotwb.Leg[i].espd_h.x=vmc[i].spd.x;
	robotwb.Leg[i].espd_h.y=vmc[i].spd.y;
    robotwb.Leg[i].espd_h.z=vmc[i].spd.z;

    robotwb.Leg[i].espd_n.x=vmc[i].spd_n.x;
	robotwb.Leg[i].espd_n.y=vmc[i].spd_n.y;
    robotwb.Leg[i].espd_n.z=vmc[i].spd_n.z;
		
    robotwb.Leg[i].jacobi[0]=vmc[i].jacobi22[0];
    robotwb.Leg[i].jacobi[2]=vmc[i].jacobi22[2];

    robotwb.Leg[i].jacobi[1]=vmc[i].jacobi22[1];
    robotwb.Leg[i].jacobi[3]=vmc[i].jacobi22[3];

    robotwb.Leg[i].jacobi_inv[0]=vmc[i].ijacobi22[0];
    robotwb.Leg[i].jacobi_inv[2]=vmc[i].ijacobi22[2];

    robotwb.Leg[i].jacobi_inv[1]=vmc[i].ijacobi22[1];
    robotwb.Leg[i].jacobi_inv[3]=vmc[i].ijacobi22[3];
	}
}

void readAllMotorPos(robotTypeDef* rob,float dt)
{
#if !RUN_WEBOTS
	static float cnt;
	uint8_t i, j;
	static char init = 0;
	dt = LIMIT(dt, 0.001, 0.1);
	float taom[4][3] = { 0 };
	float temp_sita[4][3] = { 0 };
    cnt+=dt;
    for (i = 0; i < 4; i++){//��convert_vmc_webot_data�и�ֵ��VMC
			if(leg_motor[i].connect&&
				leg_motor[i].connect_motor[0]&&
			  leg_motor[i].ready[0]){
				rob->Leg[i].sita[0]=(leg_motor[i].q_now[0]);//sita0  ������
				temp_sita[i][0]=leg_motor[i].q_now[0];
				rob->Leg[i].taom[0]=leg_motor[i].t_now[0];
			}else{//��������ʾ
				rob->Leg[i].sita[0]=0;
				rob->Leg[i].taom[0]=0;
				temp_sita[i][0]=0;			
			}
			
			if(leg_motor[i].connect&&
			  leg_motor[i].connect_motor[1]&&
			  leg_motor[i].ready[1]){
				rob->Leg[i].sita[1]=To_360_degreesw(leg_motor[i].q_now[1]);//sita1 0~360
				temp_sita[i][1]=leg_motor[i].q_now[1];	
				rob->Leg[i].taom[1]=leg_motor[i].t_now[1];
					}else{//��������ʾ
				rob->Leg[i].sita[1]=180;
				rob->Leg[i].taom[1]=0;
				temp_sita[i][1]=180;					
			}

			rob->Leg[i].sita_d[0]=Moving_Median(2*i,	3,  To_180_degreesw(temp_sita[i][0]-rob->Leg[i].sita_reg[0])/dt);
			rob->Leg[i].sita_d[1]=Moving_Median(2*i+1,3,	To_180_degreesw(temp_sita[i][1]-rob->Leg[i].sita_reg[1])/dt);

			rob->Leg[i].sita_reg[0]=temp_sita[i][0];
			rob->Leg[i].sita_reg[1]=temp_sita[i][1];
    }
#else
		static float cnt;
		uint8_t i, j;
		float taom[4][3] = { 0 };
		cnt += dt;
		for (i = 0; i < 4; i++) {
			for (j = 0; j < 2; j++) {
				rob->Leg[i].pos[j] = wb_position_sensor_get_value(posensor[2 * i + j]);
				taom[i][j] = wb_motor_get_torque_feedback(motor[2 * i + j]);
			}

			switch (i) {
			case FRw:
				rob->Leg[i].sita[0] = (-rob->Leg[i].pos[0] * 57.3);
				rob->Leg[i].sita[1] = (180 - rob->Leg[i].pos[1] * 57.3);

				rob->Leg[i].taom[0] = -wb_motor_get_torque_feedback(motor[0]);
				rob->Leg[i].taom[1] = -wb_motor_get_torque_feedback(motor[1]);
				break;
			case HRw:
				rob->Leg[i].sita[0] = (-rob->Leg[i].pos[0] * 57.3);
				rob->Leg[i].sita[1] = (180 - rob->Leg[i].pos[1] * 57.3);

				rob->Leg[i].taom[0] = -wb_motor_get_torque_feedback(motor[2]);
				rob->Leg[i].taom[1] = -wb_motor_get_torque_feedback(motor[3]);
				break;
			case FLw:
				rob->Leg[i].sita[0] = (-rob->Leg[i].pos[0] * 57.3);
				rob->Leg[i].sita[1] = (180 - rob->Leg[i].pos[1] * 57.3);

				rob->Leg[i].taom[0] = -wb_motor_get_torque_feedback(motor[4]);
				rob->Leg[i].taom[1] = -wb_motor_get_torque_feedback(motor[5]);
				break;
			case HLw:
				rob->Leg[i].sita[0] = (-rob->Leg[i].pos[0] * 57.3);
				rob->Leg[i].sita[1] = (180 - rob->Leg[i].pos[1] * 57.3);

				rob->Leg[i].taom[0] = -wb_motor_get_torque_feedback(motor[6]);
				rob->Leg[i].taom[1] = -wb_motor_get_torque_feedback(motor[7]);
				break;
			}

			DigitalLPFw(To_180_degreesw(rob->Leg[i].sita[0] - rob->Leg[i].sita_reg[0]) / dt, &rob->Leg[i].sita_d[0], 30, dt);
			DigitalLPFw(To_180_degreesw(rob->Leg[i].sita[1] - rob->Leg[i].sita_reg[1]) / dt, &rob->Leg[i].sita_d[1], 30, dt);
			if (rob->Leg[i].is_ground == 0) {
				rob->Leg[i].tao_bias[0] = -rob->Leg[i].sita_d[0] * pos_force_p.motor_i_b[0];
				rob->Leg[i].tao_bias[1] = -rob->Leg[i].sita_d[1] * pos_force_p.motor_i_b[1];
				if (i == 0&&0)
					printf("%d %f %f %f\n", i, robotwb.Leg[i].taom_output[0], rob->Leg[i].sita_d[0], rob->Leg[i].tao_bias[0]);
			}
			else {
				rob->Leg[i].tao_bias[0] = rob->Leg[i].tao_bias[1] = 0;
			}

			rob->Leg[i].sita_reg[0] = rob->Leg[i].sita[0];
			rob->Leg[i].sita_reg[1] = rob->Leg[i].sita[1];
		}
#endif
}

#if !RUN_WEBOTS
void set_motor_q(int id)
{  
    robotwb.Leg[id].tar_sita[0]=LIMIT(robotwb.Leg[id].tar_sita[0],-robotwb.Leg[id].limit_sita[0],180+robotwb.Leg[id].limit_sita[0]);
    robotwb.Leg[id].tar_sita[1]=LIMIT(robotwb.Leg[id].tar_sita[1],-robotwb.Leg[id].limit_sita[1],180+robotwb.Leg[id].limit_sita[1]);
}

void set_motor_t(int id)
{
    robotwb.Leg[id].taod[0]=LIMIT(robotwb.Leg[id].taod[0],-robotwb.Leg[id].limit_tao[0],robotwb.Leg[id].limit_tao[0]);
    robotwb.Leg[id].taod[1]=LIMIT(robotwb.Leg[id].taod[1],-robotwb.Leg[id].limit_tao[1],robotwb.Leg[id].limit_tao[1]);
    robotwb.Leg[id].taom_output[0]=robotwb.Leg[id].taod[0];//��¼�������
    robotwb.Leg[id].taom_output[1]=robotwb.Leg[id].taod[1];
	#if EN_TORQUE_CONTROL
		leg_motor[id].set_t[0]=robotwb.Leg[id].taom_output[0];
		leg_motor[id].set_t[1]=robotwb.Leg[id].taom_output[1];
		leg_motor[id].set_i[0]=leg_motor[id].set_t[0]/(leg_motor[id].t_to_i[0]+0.0000001);
		leg_motor[id].set_i[1]=leg_motor[id].set_t[1]/(leg_motor[id].t_to_i[1]+0.0000001);
	#else
		leg_motor[id].set_t[0]=0;
		leg_motor[id].set_t[1]=0;
	#endif
}
#else


void pos_control_pd(float dt)
{
	int id = 3;
	for (id = 0; id < 4; id++) {//���ԽǶ�����ϵ
	  // robotwb.Leg[id].tar_sita[0]=-0;
	  // robotwb.Leg[id].tar_sita[1]=180;
		robotwb.Leg[id].tar_sita[0] = limitw(robotwb.Leg[id].tar_sita[0], -robotwb.Leg[id].limit_sita[0], 180 + robotwb.Leg[id].limit_sita[0]);
		robotwb.Leg[id].tar_sita[1] = limitw(robotwb.Leg[id].tar_sita[1], -robotwb.Leg[id].limit_sita[1], 180 + robotwb.Leg[id].limit_sita[1]);
		robotwb.Leg[id].taod[0] = (To_180_degreesw(robotwb.Leg[id].tar_sita[0] - robotwb.Leg[id].sita[0])*robotwb.Leg[id].q_pid.kp - robotwb.Leg[id].sita_d[0] * robotwb.Leg[id].q_pid.kd) * 1;
		robotwb.Leg[id].taod[1] = (To_180_degreesw(robotwb.Leg[id].tar_sita[1] - robotwb.Leg[id].sita[1])*robotwb.Leg[id].q_pid.kp - robotwb.Leg[id].sita_d[1] * robotwb.Leg[id].q_pid.kd) * 1;
		//printf("leg=%d s0=%f %f s1=%f %f ds=%f %f\n",id,robotwb.Leg[id].tar_sita[0],robotwb.Leg[id].sita[0],robotwb.Leg[id].tar_sita[1],robotwb.Leg[id].sita[1],robotwb.Leg[id].sita_d[0],robotwb.Leg[id].sita_d[1]);
		set_motor_t(id);
	}
}


void set_motor_q(int id)
{
	robotwb.Leg[id].tar_sita[0] = limitw(robotwb.Leg[id].tar_sita[0], -robotwb.Leg[id].limit_sita[0], 180 + robotwb.Leg[id].limit_sita[0]);
	robotwb.Leg[id].tar_sita[1] = limitw(robotwb.Leg[id].tar_sita[1], -robotwb.Leg[id].limit_sita[1], 180 + robotwb.Leg[id].limit_sita[1]);
	switch (id) {
	case FRw:
		wb_motor_set_position(motor[0], rad(-robotwb.Leg[id].tar_sita[0]));
		wb_motor_set_position(motor[1], rad(To_180_degreesw(180 - robotwb.Leg[id].tar_sita[1])));
		break;
	case HRw:
		wb_motor_set_position(motor[2], rad(-robotwb.Leg[id].tar_sita[0]));
		wb_motor_set_position(motor[3], rad(To_180_degreesw(180 - robotwb.Leg[id].tar_sita[1])));
		break;
	case FLw:
		wb_motor_set_position(motor[4], rad(-robotwb.Leg[id].tar_sita[0]));
		wb_motor_set_position(motor[5], rad(To_180_degreesw(180 - robotwb.Leg[id].tar_sita[1])));
		break;
	case HLw:
		wb_motor_set_position(motor[6], rad(-robotwb.Leg[id].tar_sita[0]));
		wb_motor_set_position(motor[7], rad(To_180_degreesw(180 - robotwb.Leg[id].tar_sita[1])));
		break;
	}
}

void set_motor_t(int id)
{
	robotwb.Leg[id].taod[0] = limitw(robotwb.Leg[id].taod[0], -robotwb.Leg[id].limit_tao[0], robotwb.Leg[id].limit_tao[0]);
	robotwb.Leg[id].taod[1] = limitw(robotwb.Leg[id].taod[1], -robotwb.Leg[id].limit_tao[1], robotwb.Leg[id].limit_tao[1]);
	robotwb.Leg[id].taom_output[0] = robotwb.Leg[id].taod[0];
	robotwb.Leg[id].taom_output[1] = robotwb.Leg[id].taod[1];
	//printf("%f %f\n", robotwb.Leg[id].taod[0], robotwb.Leg[id].taod[1]);
	switch (id) {
	case FRw:
		wb_motor_set_torque(motor[0], -robotwb.Leg[id].taod[0]);
		wb_motor_set_torque(motor[1], -robotwb.Leg[id].taod[1]);
		break;
	case HRw:
		wb_motor_set_torque(motor[2], -robotwb.Leg[id].taod[0]);
		wb_motor_set_torque(motor[3], -robotwb.Leg[id].taod[1]);
		break;
	case FLw:
		wb_motor_set_torque(motor[4], -robotwb.Leg[id].taod[0]);
		wb_motor_set_torque(motor[5], -robotwb.Leg[id].taod[1]);
		break;
	case HLw:
		wb_motor_set_torque(motor[6], -robotwb.Leg[id].taod[0]);
		wb_motor_set_torque(motor[7], -robotwb.Leg[id].taod[1]);
		break;
	}
}

void updateJoy(void)//user_xd_d button
{
	float axis_max = 32768.0;
	float FLT_RC = 1;
	float T = 0.005;
	float rate_yaw_w[2];
	int axis_dead = 200;
	int axis = wb_joystick_get_number_of_axes();
	int povs = wb_joystick_get_number_of_povs();
	static float timer_rc = 0;
	static int flag_rc = 0;
	if (wb_joystick_is_connected())
	{
		// printf("joy_is_connected Axis=%d %d\n",wb_joystick_get_number_of_axes(),wb_joystick_get_number_of_povs());
		for (int i = 0; i < axis; i++)
		{
			int axis_v = 0;// dead(wb_joystick_get_axis_value(i), axis_dead);
			float rc_temp = (float)axis_v / axis_max;
			//printf("[%d]axis = %f\n",i,rc_temp);
			if (i == 0)
				robotwb.ocu.rc_spd_w[Xrw] = -rc_temp;

			if (i == 1)
				robotwb.ocu.rc_spd_w[Yrw] = rc_temp;

			if (i == 2)
				robotwb.ocu.rc_att_w[PITrw] = -rc_temp;

			if (i == 3)
				robotwb.ocu.rc_att_w[ROLrw] = rc_temp;

			if (i == 4)
				rate_yaw_w[0] = rc_temp;

			if (i == 5)
				rate_yaw_w[1] = rc_temp;
		}

		robotwb.ocu.rate_yaw_w = rate_yaw_w[0] - rate_yaw_w[1];
		for (int i = 0; i < povs; i++)
		{
			int povs_v = (wb_joystick_get_pov_value(i));
			// printf("[%d]pov = %f\n",i,povs_v);

		}
		int button = wb_joystick_get_pressed_button();
		//printf("button = %d\n",button);
		robotwb.ocu.key_y = 0;
		robotwb.ocu.key_x = 0;
		robotwb.ocu.key_b = 0;
		robotwb.ocu.key_a = 0;
		robotwb.ocu.key_st = 0;
		robotwb.ocu.key_back = 0;
		robotwb.ocu.key_ll = 0;
		robotwb.ocu.key_rr = 0;

		if (button == 11)
			robotwb.ocu.key_y = 1;
		if (button == 10)
			robotwb.ocu.key_x = 1;
		if (button == 9)
			robotwb.ocu.key_b = 1;
		if (button == 8)
			robotwb.ocu.key_a = 1;
		if (button == 4)
			robotwb.ocu.key_ll = 1;
		if (button == 5)
			robotwb.ocu.key_rr = 1;
		if (button == 0)
			robotwb.ocu.key_st = 1;
		if (button == 1)
			robotwb.ocu.key_back = 1;
	}
	else
	{
#if NO_RC
		if (vmc_all.gait_mode == TROT)
			timer_rc += 0.005;
		else
			timer_rc = 0;
		if (timer_rc > FORWARD_REPEAT_TIME)
		{
			timer_rc = 0;
			flag_rc = !flag_rc;
		}
#if 1
		//DigitalLPF(ocu.rc_spd_w[Xr] * MAX_SPD, &vmc_all.tar_spd.x, FLT_RC * 4, T);
		//DigitalLPF(ocu.rc_spd_w[Yr] * MAX_SPD, &vmc_all.tar_spd.y, FLT_RC * 4 / 2, T);
		//DigitalLPF(ocu.rate_yaw_w*MAX_SPD_RAD, &vmc_all.tar_spd.z, FLT_RC * 4, T);
		//DigitalLPF(-ocu.rc_att_w[PITr] * vmc_all.param.MAX_PIT, &vmc_all.tar_att[PITr], FLT_RC * 4, T);
		//DigitalLPF(ocu.rc_att_w[ROLr] * vmc_all.param.MAX_ROL, &vmc_all.tar_att[ROLr], FLT_RC * 4, T);

		if (!flag_rc)
			DigitalLPFw(FORWARD_SPD, &vmc_all.tar_spd.x, 0.3, T);
		else
			DigitalLPFw(-FORWARD_SPD, &vmc_all.tar_spd.x, 0.3, T);

		vmc_all.tar_spd.x *= EN_AUTO_FORWARD_TROT;
		vmc_all.tar_spd.z = -60 * EN_AUTO_ROTATE_TROT;

#endif
#endif
	}

}

void updateKey(void) {
	int key = wb_keyboard_get_key();
	if (key > 0) {
		printf("Key board==%d\n", key);
	}
}


FILE *fp1, *fp2, *fp3;
#define LEN_RECORD 80//��¼λ��
float data_record1[LEN_RECORD], data_record2[LEN_RECORD], data_record3[LEN_RECORD];
void recorder(float dt) {
	static int state = 0;
	int i = 0;
	switch (state)
	{
	case 0:
		fp1 = fopen(".\\Data\\file1.txt", "w");
		fp2 = fopen(".\\Data\\file2.txt", "w");
		fp3 = fopen(".\\Data\\file3.txt", "w");

		if (fp1 != NULL) {
			printf("---------------------------------Record Ready---------------------------------\n");
			state++;
		}
		break;
	case 1:
		for (i = 0; i < LEN_RECORD; i++)
			fprintf(fp1, "%.6f ",//��̬����
				data_record1[i]);
		fprintf(fp1, "\n");

		for (i = 0; i < LEN_RECORD; i++)
			fprintf(fp2, "%.6f ",//������
				data_record2[i]);
		fprintf(fp2, "\n");

		for (i = 0; i < LEN_RECORD; i++)
			fprintf(fp3, "%.6f ",//�����˶�
				data_record3[i]);
		fprintf(fp3, "\n");
		break;
	case 2:
		fclose(fp1); fclose(fp2); fclose(fp3);
		printf("---------------------------------Record Finish--------------------------\n");
		break;
	}

}

void record_thread(char sel, float dt)
{
	int cnt = 0;
	switch (sel) {
	case 0:
		cnt = 0;
		data_record1[cnt++] = robotwb.exp_att.pitch;
		data_record1[cnt++] = robotwb.now_att.pitch;

		data_record1[cnt++] = robotwb.exp_att.roll;
		data_record1[cnt++] = robotwb.now_att.roll;

		data_record1[cnt++] = robotwb.exp_att.yaw;
		data_record1[cnt++] = robotwb.now_att.yaw;

		data_record1[cnt++] = robotwb.now_rate.pitch;
		data_record1[cnt++] = robotwb.now_rate.roll;
		data_record1[cnt++] = robotwb.now_rate.yaw;

		data_record1[cnt++] = robotwb.exp_pos_n.x;
		data_record1[cnt++] = vmc_all.pos_n.x;// robotwb.cog_pos_n.x;
		data_record1[cnt++] = vmc_all.spd_n.x; //robotwb.cog_spd_n.x;

		data_record1[cnt++] = robotwb.exp_pos_n.z;
		data_record1[cnt++] = vmc_all.pos_n.z;// robotwb.cog_pos_n.z;
		data_record1[cnt++] = vmc_all.spd_n.z;// robotwb.cog_spd_n.z;

		data_record1[cnt++] = vmc_all.ground_att_est[PITr];// robotwb.ground_att_cmd[PITrw];
		data_record1[cnt++] = vmc_all.ground_att_est[ROLr];//robotwb.ground_att_cmd[ROLrw];

		data_record1[cnt++] = robotwb.exp_spd_n.x;// -robotwb.exp_spd_b.x;
		data_record1[cnt++] = robotwb.exp_spd_n.z;//robotwb.exp_spd_b.z;
		//printf(" vmc_all.pos_n.z=%f %f\n", vmc_all.pos_n.z, robotwb.Leg[1].epos_h.z);
		cnt = 0;
		data_record2[cnt++] = robotwb.Leg[0].tar_force_dis_n.x;
		data_record2[cnt++] = robotwb.Leg[0].force_est_n_output.x;
		data_record2[cnt++] = robotwb.Leg[0].tar_force_dis_n.z;
		data_record2[cnt++] = robotwb.Leg[0].force_est_n_output.z;
		data_record2[cnt++] = robotwb.Leg[1].tar_force_dis_n.x;
		data_record2[cnt++] = robotwb.Leg[1].force_est_n_output.x;
		data_record2[cnt++] = robotwb.Leg[1].tar_force_dis_n.z;
		data_record2[cnt++] = robotwb.Leg[1].force_est_n_output.z;
		data_record2[cnt++] = robotwb.Leg[2].tar_force_dis_n.x;
		data_record2[cnt++] = robotwb.Leg[2].force_est_n_output.x;
		data_record2[cnt++] = robotwb.Leg[2].tar_force_dis_n.z;
		data_record2[cnt++] = robotwb.Leg[2].force_est_n_output.z;
		data_record2[cnt++] = robotwb.Leg[3].tar_force_dis_n.x;
		data_record2[cnt++] = robotwb.Leg[3].force_est_n_output.x;
		data_record2[cnt++] = robotwb.Leg[3].tar_force_dis_n.z;
		data_record2[cnt++] = robotwb.Leg[3].force_est_n_output.z;

		data_record2[cnt++] = robotwb.Leg[0].is_ground;
		data_record2[cnt++] = robotwb.Leg[0].is_touch_est;
		data_record2[cnt++] = robotwb.Leg[1].is_ground;
		data_record2[cnt++] = robotwb.Leg[1].is_touch_est;
		data_record2[cnt++] = robotwb.Leg[2].is_ground;
		data_record2[cnt++] = robotwb.Leg[2].is_touch_est;
		data_record2[cnt++] = robotwb.Leg[3].is_ground;
		data_record2[cnt++] = robotwb.Leg[3].is_touch_est;

		float trig_state[4] = { 0 };
		for (int i = 0; i < 4; i++) {
			trig_state[i] = robotwb.Leg[i].trig_state;
			if (robotwb.Leg[i].trig_state == 3)
				trig_state[i] = 1.5;
			else if (robotwb.Leg[i].trig_state == 4)
				trig_state[i] = 1;
		}
		data_record2[cnt++] = trig_state[0];
		data_record2[cnt++] = trig_state[1];
		data_record2[cnt++] = trig_state[2];
		data_record2[cnt++] = trig_state[3];

		data_record2[cnt++] = robotwb.Leg[0].tar_sita[0];
		data_record2[cnt++] = robotwb.Leg[0].tar_sita[1];
		data_record2[cnt++] = robotwb.Leg[0].sita[0];
		data_record2[cnt++] = robotwb.Leg[0].sita[1];
		data_record2[cnt++] = vmc[0].param.tar_epos_h.x;// robotwb.Leg[0].tar_epos_h.x;
		data_record2[cnt++] = vmc[0].param.tar_epos_h.z;// robotwb.Leg[0].tar_epos_h.z;
		data_record2[cnt++] = robotwb.Leg[0].epos_h.x;
		data_record2[cnt++] = robotwb.Leg[0].epos_h.z;
		//printf("%f\n", robotwb.Leg[0].tar_epos_h.z);
		data_record2[cnt++] = robotwb.Leg[1].tar_sita[0];
		data_record2[cnt++] = robotwb.Leg[1].tar_sita[1];
		data_record2[cnt++] = robotwb.Leg[1].sita[0];
		data_record2[cnt++] = robotwb.Leg[1].sita[1];
		data_record2[cnt++] = vmc[1].param.tar_epos_h.x;// robotwb.Leg[0].tar_epos_h.x;
		data_record2[cnt++] = vmc[1].param.tar_epos_h.z;// robotwb.Leg[0].tar_epos_h.z;
		data_record2[cnt++] = robotwb.Leg[1].epos_h.x;
		data_record2[cnt++] = robotwb.Leg[1].epos_h.z;

		data_record2[cnt++] = robotwb.Leg[2].tar_sita[0];
		data_record2[cnt++] = robotwb.Leg[2].tar_sita[1];
		data_record2[cnt++] = robotwb.Leg[2].sita[0];
		data_record2[cnt++] = robotwb.Leg[2].sita[1];
		data_record2[cnt++] = vmc[2].param.tar_epos_h.x;// robotwb.Leg[0].tar_epos_h.x;
		data_record2[cnt++] = vmc[2].param.tar_epos_h.z;// robotwb.Leg[0].tar_epos_h.z;
		data_record2[cnt++] = robotwb.Leg[2].epos_h.x;
		data_record2[cnt++] = robotwb.Leg[2].epos_h.z;

		data_record2[cnt++] = robotwb.Leg[3].tar_sita[0];
		data_record2[cnt++] = robotwb.Leg[3].tar_sita[1];
		data_record2[cnt++] = robotwb.Leg[3].sita[0];
		data_record2[cnt++] = robotwb.Leg[3].sita[1];
		data_record2[cnt++] = vmc[3].param.tar_epos_h.x;// robotwb.Leg[0].tar_epos_h.x;
		data_record2[cnt++] = vmc[3].param.tar_epos_h.z;// robotwb.Leg[0].tar_epos_h.z;
		data_record2[cnt++] = robotwb.Leg[3].epos_h.x;
		data_record2[cnt++] = robotwb.Leg[3].epos_h.z;

		cnt = 0;
		data_record3[cnt++] = robotwb.now_att.roll / 57.3;
		data_record3[cnt++] = adrc_outter_att[1].z1;
		data_record3[cnt++] = robotwb.now_rate.roll / 57.3;
		data_record3[cnt++] = adrc_outter_att[1].z2;
		data_record3[cnt++] = adrc_outter_att[1].z3;
		break;
	}
}
#endif