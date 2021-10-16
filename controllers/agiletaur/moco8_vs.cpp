// File:          moco8_vs.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/robot.hpp>
#include <Eigen/Dense>
#include <qpOASES.hpp>
#include "wbInterface.h"
#include "optimaize.h"
#include "locomotion_header.h"
#include "gait_math.h"
#include "adrc.h"
#include "convexMPC_interface.h"
#include "common_types.h"
#include "SolverMPC.h"
#include "cppTypes.h"

using namespace Eigen;
using namespace std;
using namespace qpOASES;
// All the webots classes are defined in the "webots" namespace
using namespace webots;

Robot *robot = new Robot();
int testQP()
{
		/* Setup data of first QP. */
	real_t H[2 * 2] = { 1.0, 0.0, 0.0, 0.5 };
	real_t A[1 * 2] = { 1.0, 1.0 };
	real_t g[2] = { 1.5, 1.0 };
	real_t lb[2] = { 0.5, -2.0 };
	real_t ub[2] = { 5.0, 2.0 };
	real_t lbA[1] = { -1.0 };
	real_t ubA[1] = { 2.0 };
	/* Setup data of second QP. */
	real_t g_new[2] = { 1.0, 1.5 };
	real_t lb_new[2] = { 0.0, -1.0 };
	real_t ub_new[2] = { 5.0, -0.5 };
	real_t lbA_new[1] = { -2.0 };
	real_t ubA_new[1] = { 1.0 };
	/* Setting up QProblem object. */
	QProblem example(2, 1);
	/* Solve first QP. */
	int_t nWSR = 10;
	example.init(H, g, A, lb, ub, lbA, ubA, nWSR);
	/* Solve second QP. */
	nWSR = 10;
	example.hotstart(g_new, lb_new, ub_new, lbA_new, ubA_new, nWSR);
	/* Get and print solution of second QP. */
	real_t xOpt[2];
	example.getPrimalSolution(xOpt);
	//printf("\n xOpt = [ %e, %e ]; objVal = %e\n\n",xOpt[0], xOpt[1], example.getObjVal());
	return 0;
}

int testQP2()
{
	static int init = 0;

		/* Setup data of first QP. */
	real_t H[2 * 2] = { 1.0, 0.0, 0.0, 0.5 };
	real_t A[1 * 2] = { 1.0, 1.0 };
	real_t g[2] = { 1.5, 1.0 };
	real_t lb[2] = { 0.5, -2.0 };
	real_t ub[2] = { 5.0, 2.0 };
	real_t lbA[1] = { -1.0 };
	real_t ubA[1] = { 2.0 };
	/* Setup data of second QP. */
	real_t g_new[2] = { 1.0, 1.5 };
	real_t lb_new[2] = { 0.0, -1.0 };
	real_t ub_new[2] = { 5.0, -0.5 };
	real_t lbA_new[1] = { -2.0 };
	real_t ubA_new[1] = { 1.0 };
	/* Setting up QProblem object. */

	static QProblem example(2, 1);
	/* Solve first QP. */
	int_t nWSR = 10;
	if (!init) {
		init = 1;
		example.init(H, g, A, lb, ub, lbA, ubA, nWSR);
		printf("QP Init!\n");
	}
	/* Solve second QP. */
	nWSR = 10;
	example.hotstart(g_new, lb_new, ub_new, lbA_new, ubA_new, nWSR);
	/* Get and print solution of second QP. */
	real_t xOpt[2];
	example.getPrimalSolution(xOpt);
	printf("\n xOpt = [ %e, %e ]; objVal = %e\n\n",xOpt[0], xOpt[1], example.getObjVal());
	return 0;
}

void test_eigen()
{

	Eigen::MatrixXd m = MatrixXd::Random(3, 3);              //随机生成3*3的double型矩阵
	m = (m + MatrixXd::Constant(3, 3, 1.2)) * 50;      //MatrixXd::Constant(3,3,1.2)表示生成3*3的double型矩阵，该矩阵所有元素均为1.2
	cout << "m =" << endl << m << endl;
	VectorXd v(3);        // 定义v为3*1的double型向量
	v << 1, 2, 3;         // 向量赋值
	cout << "v =" << endl << v << endl;
	cout << "m * v =" << endl << m * v << endl;
}


void Duty_VMC_Gait(float T)//VMC顶层控制
{
	locomotion_sfm(T);
}

void Duty_PForce_Control(float T)//底层采集  运动学解算 //2ms
{
	char i;
	readAllMotorPos(&robotwb, T);//读取角度 

	subscribe_webot_to_vmc1(T);//转换输出bug  之前在20ms线程导致状态反馈频率低 读取角度

#if !TEST12
	for (i = 0; i < 4; i++) {
		estimate_end_state_new(&vmc[i], T);	//运动学正解
	}

	for (i = 0; i < 4; i++) {
		cal_jacobi_new(&vmc[i]);	//计算雅克比

		cal_invjacobi(&vmc[i]);		//计算雅克比逆
	}

	publish_vmc_to_webot(T);//运动学值赋值给robot结构体 仅赋值
#else
	for (i = 0; i < 4; i++) {
		cal_jacobi_new(&vmc[i]);	//计算雅克比

		cal_invjacobi(&vmc[i]);		//计算雅克比逆
	}
	publish_vmc_to_webot(T);//运动学值赋值给robot结构体 仅赋值
#endif
	estimate_GRF(T);//估计足底力

	touchdown_check(&robotwb, T);//判断着地

	subscribe_webot_to_vmc3(T);//赋值VMC 着地判断

	Duty_VMC_Gait(T);//摆动控制

	subscribe_webot_to_vmc2(T);//赋值Robot 着地判断结果  和 支撑摆动参数

	if (ocu.cmd_robot_state >= 2) {
		if (vmc_all.gait_mode == TROT || vmc_all.gait_mode == F_TROT) 
			//force_control_and_dis_trot(T);
			force_control_and_dis_trot_release(T);
		else
			force_control_and_dis_stand(T);
			//force_control_and_dis_stand_release(T);
		record_thread(0, T);
		record_thread(1, T);
		recorder(T);
	}
}

void Duty_Att_Fushion(float T)//姿态解算 100Hz
{
	u8 i;
	static u8 init;
	static u16 cnt_init;
	static float timer_baro;
	static float att_rt_use[3];

	subscribe_imu_to_webot(&robotwb, T);//赋值给robot结构体

#if TEST12
		for (i = 0; i < 4; i++) {
			estimate_end_state_new(&vmc[i], T);	//运动学正解
		}
		publish_vmc_to_webot(T);//运动学值赋值给robot结构体 仅赋值
#endif

		state_estimator(T);//估计质心状态

		estimate_ground_att(T);	//地形估计

		subscribe_webot_to_vmc4(T);//摆动与地形角
#if USE_FPOS_CONTROL
		body_servo_control(T);//VMC虚拟伺服
#endif

}

Vec3<float> Fr_des[4];

int main(int argc, char **argv) {
  // create the Robot instance.
	static float init_cnt = 0;
	static int init_done = 0;
	int i = 0;
	static float timer[10] = { 0 };
	float dT = TIME_STEP / 1000.;
	printf("Code Editor is Vs2017\n");
	get_license_moco();
	check_lisence_moco(1,1,1);
	//testQP();
	//test_eigen();
	webots_device_init();
	ADRC_Init();
	setup_problem(TIME_STEP*0.001, 10, 0.4, 80);

	while (wb_robot_step(TIME_STEP) != -1) {//5MS
		init_cnt += dT;
		if (init_done) {
			updateJoy();

			Duty_Att_Fushion(dT);//状态估计 + VMC力控		

			Duty_PForce_Control(dT);
		}
		else
		{
			readAllMotorPos(&robotwb, dT);//读取角度 
			subscribe_imu_to_webot(&robotwb, dT);//赋值给robot结构体
			for (i = 0; i < 4; i++) {
				estimate_end_state_new(&vmc[i], dT);	//运动学正解
			}

			for (i = 0; i < 4; i++) {
				cal_jacobi_new(&vmc[i]);	//计算雅克比

				cal_invjacobi(&vmc[i]);		//计算雅克比逆
			}

			publish_vmc_to_webot(dT);//运动学值赋值给robot结构体 仅赋值
			if (init_cnt >= 0.025)
				init_done = 1;
		}
	}

  delete robot;
  return 0;
}
