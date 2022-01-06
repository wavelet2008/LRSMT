#include "include.h"
#include "locomotion_header.h"
#include "math.h"
#include "eso.h"
#include "gait_math.h"
#if !RUN_WEBOTS
#include "arm_math.h"
#endif
//----------------------����ת����
void mat_trans(float src[3][3],float dis[3][3])
{
	char i,j;
	float temp;
#if 0
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
		{
			if (i > j)
			{
				temp = src[i][j];
				dis[i][j] = src[j][i];
				dis[j][i] = temp;
			}
		}
#else
	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			dis[j][i] = src[i][j];
		}
	}
#endif
}

void vect3_2_cross(Vect3 w,float cross[3][3])
{
	cross[0][0] = 0;    cross[0][1] = -w.z; cross[0][2] = w.y;
	cross[1][0] = w.z ; cross[1][1] = 0;    cross[1][2] = -w.x;
	cross[2][0] = -w.y; cross[2][1] = w.x;   cross[2][2] = 0;
}

void matrx33_mult_vect3(float matrix[3][3],Vect3 src,Vect3* dir)
{
	dir->x = matrix[0][0] * src.x + matrix[0][1] * src.y + matrix[0][2] * src.z;
	dir->y = matrix[1][0] * src.x + matrix[1][1] * src.y + matrix[1][2] * src.z;
	dir->z = matrix[2][0] * src.x + matrix[2][1] * src.y + matrix[2][2] * src.z;
}

void matrx33_mult_matrx33(float matrix1[3][3], float matrix2[3][3], float matrixo[3][3])
{
	int i, j, k;
	float lSum=0;

	/*Ƕ��ѭ������������m*p����ÿ��Ԫ��*/
	for (i = 0; i < 3; i++){
		for (j = 0; j < 3; j++)
		{
			/*���վ���˷��Ĺ�������������i*jԪ��*/
			lSum = 0;
			for (k = 0; k < 3; k++)
				lSum += matrix1[i][k] * matrix2[k][j];
			matrixo[i][j] = lSum;
		}
	}
}


void converV_n_to_bw(Vect3 vn,Vect3* vb){
	float temp=0;
	vb->x  = robotwb.Rn_b[0][0]*vn.x + robotwb.Rn_b[0][1]*vn.y + robotwb.Rn_b[0][2]*vn.z;
	vb->y  = robotwb.Rn_b[1][0]*vn.x + robotwb.Rn_b[1][1]*vn.y + robotwb.Rn_b[1][2]*vn.z;
	temp   = robotwb.Rn_b[2][0]*vn.x + robotwb.Rn_b[2][1]*vn.y + robotwb.Rn_b[2][2]*vn.z;
	#if F_CONTROL_WITH_ROLL
	vb->z  = temp/cosd(LIMIT(robotwb.now_att.roll,-10,10));
	#else
	vb->z  = temp ;
	#endif
}

void converV_b_to_legw(int id,Vect3 vb,Vect3* vl){
    vl->x = vb.x- Hw/2*robotwb.Leg[id].flag_fb;
    vl->y = vb.y- Www/2*robotwb.Leg[id].flag_rl;
    vl->z = vb.z;
}

void converV_b_to_leg_ow(char leg,float xb,float yb,float zb,float *xl,float *yl,float *zl)
{
	*xl = xb- Hw/2*robotwb.Leg[leg].flag_fb;
	*yl = yb- Www/2*robotwb.Leg[leg].flag_rl;
	*zl = zb;
}

void converV_leg_to_bw(int id,Vect3 vl,Vect3* vb){
    vb->x = vl.x+ Hw/2*robotwb.Leg[id].flag_fb;
    vb->y = vl.y+ Www/2*robotwb.Leg[id].flag_rl;
    vb->z = vl.z;
}

void converV_b_to_nw(Vect3 vb,Vect3* vn){
    vn->x =  robotwb.Rb_n[0][0]*vb.x +  robotwb.Rb_n[0][1]*vb.y +  robotwb.Rb_n[0][2]*vb.z;
    vn->y =  robotwb.Rb_n[1][0]*vb.x +  robotwb.Rb_n[1][1]*vb.y +  robotwb.Rb_n[1][2]*vb.z;
    vn->z =  robotwb.Rb_n[2][0]*vb.x +  robotwb.Rb_n[2][1]*vb.y +  robotwb.Rb_n[2][2]*vb.z;
}

void force_n_to_bw(Vect3 fn,Vect3* fb){
	float temp=0;
#if 0
	fb->x = robotwb.Rn_b[0][0] * fn.x + robotwb.Rn_b[0][1] * fn.x + robotwb.Rn_b[0][2] * fn.x;
	fb->y = robotwb.Rn_b[1][0] * fn.y + robotwb.Rn_b[1][1] * fn.y + robotwb.Rn_b[1][2] * fn.y;
	temp =  robotwb.Rn_b[2][0] * fn.z + robotwb.Rn_b[2][1] * fn.z + robotwb.Rn_b[2][2] * fn.z;
#if F_CONTROL_WITH_ROLL
	fb->z = temp / cosd(LIMIT(robotwb.now_att.roll, -10, 10));
#else
	fb->z = temp;
#endif
#elif 1
	fb->x = robotwb.Rn_b[0][0] * fn.x + robotwb.Rn_b[0][1] * fn.y + robotwb.Rn_b[0][2] * fn.z;
	fb->y = robotwb.Rn_b[1][0] * fn.x + robotwb.Rn_b[1][1] * fn.y + robotwb.Rn_b[1][2] * fn.y;
	temp =  robotwb.Rn_b[2][0] * fn.x + robotwb.Rn_b[2][1] * fn.y + robotwb.Rn_b[2][2] * fn.z;
#if F_CONTROL_WITH_ROLL
	fb->z = temp / cosd(LIMIT(robotwb.now_att.roll, -10, 10));
#else
	fb->z = temp;
#endif
#else
	fb->x =  robotwb.Rn_b[0][0]*fn.x + robotwb.Rn_b[0][1]*fn.x + robotwb.Rn_b[0][2]*fn.x;
	fb->y =  robotwb.Rn_b[1][0]*fn.x + robotwb.Rn_b[1][1]*fn.y + robotwb.Rn_b[1][2]*fn.z;
	temp  =  robotwb.Rn_b[2][0]*fn.x + robotwb.Rn_b[2][1]*fn.z + robotwb.Rn_b[2][2]*fn.z;
	#if F_CONTROL_WITH_ROLL
	fb->z  = temp/cosd(LIMIT(robotwb.now_att.roll,-10,10));
	#else
	fb->z  = temp ;
	#endif
#endif
}

int flag_gf[2] = { -1,-1 };
void converV_n_to_gw(Vect3 fn, Vect3* fg)
{
	float Rn_g[3][3] = { 0 };
	float att_rt_use[3] = { flag_gf[0] * vmc_all.ground_att_est[PITr],flag_gf[1] * vmc_all.ground_att_est[ROLr],0 };
	att_rt_use[0] = limitw(att_rt_use[0], -6, 6);
	att_rt_use[1] = limitw(att_rt_use[1], -6, 6);
	Rn_g[0][0] = cosd(-att_rt_use[PITr])*cosd(-att_rt_use[YAWr]);
	Rn_g[1][0] = -cosd(-att_rt_use[ROLr])*sind(-att_rt_use[YAWr]) + sind(-att_rt_use[PITr])*sind(-att_rt_use[ROLr])*cosd(-att_rt_use[YAWr]);
	Rn_g[2][0] = sind(-att_rt_use[ROLr])*sind(-att_rt_use[YAWr]) + cosd(-att_rt_use[ROLr])*sind(-att_rt_use[PITr])*cosd(-att_rt_use[YAWr]);
	Rn_g[0][1] = cosd(-att_rt_use[PITr])*sind(-att_rt_use[YAWr]);
	Rn_g[1][1] = cosd(-att_rt_use[ROLr])*cosd(-att_rt_use[YAWr]) + sind(-att_rt_use[ROLr])*sind(-att_rt_use[PITr])*sind(-att_rt_use[YAWr]);
	Rn_g[2][1] = -sind(-att_rt_use[ROLr])*cosd(-att_rt_use[YAWr]) + cosd(-att_rt_use[ROLr])*sind(-att_rt_use[PITr])*sind(-att_rt_use[YAWr]);
	Rn_g[0][2] = -sind(-att_rt_use[PITr]);
	Rn_g[1][2] = sind(-att_rt_use[ROLr])*cosd(-att_rt_use[PITr]);
	Rn_g[2][2] = cosd(-att_rt_use[ROLr])*cosd(-att_rt_use[PITr]);
	fg->x = Rn_g[0][0] * fn.x + Rn_g[0][1] * fn.y + Rn_g[0][2] * fn.z;
	fg->y = Rn_g[1][0] * fn.x + Rn_g[1][1] * fn.y + Rn_g[1][2] * fn.z;
	fg->z = Rn_g[2][0] * fn.x + Rn_g[2][1] * fn.y + Rn_g[2][2] * fn.z;
}


void converV_b_to_n_RTw(float RT[3][3], float yaw, float xb,float yb,float zb,float *xn,float *yn,float *zn)
{
	float x,y,z;
	x = RT[0][0]*xb + RT[0][1]*yb + RT[0][2]*zb;
	y = RT[1][0]*xb + RT[1][1]*yb + RT[1][2]*zb;
	z = RT[2][0]*xb + RT[2][1]*yb + RT[2][2]*zb;
	
	*xn=x*cosdw(yaw)-y*sindw(yaw);
	*yn=x*sindw(yaw)+y*cosdw(yaw);
	*zn=z;
}

void converV_n_to_b_w_yaWww(float yaw,float xn,float yn,float zn,float *xb,float *yb,float *zb)//unuse
{
	float x,y,z;
	x = robotwb.Rn_b[0][0]*xn + robotwb.Rn_b[0][1]*yn + robotwb.Rn_b[0][2]*zn;
	y = robotwb.Rn_b[1][0]*xn + robotwb.Rn_b[1][1]*yn + robotwb.Rn_b[1][2]*zn;
	z = robotwb.Rn_b[2][0]*xn + robotwb.Rn_b[2][1]*yn + robotwb.Rn_b[2][2]*zn;
	
	*xb=x*cosdw(yaw)-y*sindw(yaw);
	*yb=x*sindw(yaw)+y*cosdw(yaw);
	*zb=z;
}

void converV_n_to_b(float xn,float yn,float zn,float *xb,float *yb,float *zb)
{
	*xb = vmc_all.Rn_b[0][0]*xn + vmc_all.Rn_b[0][1]*yn + vmc_all.Rn_b[0][2]*zn;
	*yb = vmc_all.Rn_b[1][0]*xn + vmc_all.Rn_b[1][1]*yn + vmc_all.Rn_b[1][2]*zn;
	*zb = vmc_all.Rn_b[2][0]*xn + vmc_all.Rn_b[2][1]*yn + vmc_all.Rn_b[2][2]*zn;
}

void converV_n_to_b_RT(float RT[3][3],float yaw,float xn,float yn,float zn,float *xb,float *yb,float *zb)
{
	float x,y,z;
	x = RT[0][0]*xn + RT[0][1]*yn + RT[0][2]*zn;
	y = RT[1][0]*xn + RT[1][1]*yn + RT[1][2]*zn;
	z = RT[2][0]*xn + RT[2][1]*yn + RT[2][2]*zn;
	
	*xb=x*cosd(yaw)-y*sind(yaw);
	*yb=x*sind(yaw)+y*cosd(yaw);
	*zb=z;
}

void converV_n_to_b_w_yaw(float yaw,float xn,float yn,float zn,float *xb,float *yb,float *zb)
{
	float x,y,z;
	x = vmc_all.Rn_b[0][0]*xn + vmc_all.Rn_b[0][1]*yn + vmc_all.Rn_b[0][2]*zn;
	y = vmc_all.Rn_b[1][0]*xn + vmc_all.Rn_b[1][1]*yn + vmc_all.Rn_b[1][2]*zn;
	z = vmc_all.Rn_b[2][0]*xn + vmc_all.Rn_b[2][1]*yn + vmc_all.Rn_b[2][2]*zn;
	
	*xb=x*cosd(yaw)-y*sind(yaw);
	*yb=x*sind(yaw)+y*cosd(yaw);
	*zb=z;
}

void converV_b_to_leg(char leg,float xb,float yb,float zb,float *xl,float *yl,float *zl)
{
	*xl = xb- vmc_all.H/2*vmc[leg].flag_fb;
	*yl = yb- vmc_all.W/2*vmc[leg].flag_rl;
	*zl = zb;
}

void converV_leg_to_b(char leg,float xl,float yl,float zl,float *xb,float *yb,float *zb)
{
	*xb = xl+ vmc_all.H/2*vmc[leg].flag_fb;
	*yb = yl+ vmc_all.W/2*vmc[leg].flag_rl;
	*zb = zl;
}

void converV_b_to_n(float xb,float yb,float zb,float *xn,float *yn,float *zn)
{
	*xn = vmc_all.Rb_n[0][0]*xb + vmc_all.Rb_n[0][1]*yb + vmc_all.Rb_n[0][2]*zb;
	*yn = vmc_all.Rb_n[1][0]*xb + vmc_all.Rb_n[1][1]*yb + vmc_all.Rb_n[1][2]*zb;
	*zn = vmc_all.Rb_n[2][0]*xb + vmc_all.Rb_n[2][1]*yb + vmc_all.Rb_n[2][2]*zb;
}

void converV_b_to_n_RT(float RT[3][3], float yaw, float xb,float yb,float zb,float *xn,float *yn,float *zn)
{
	float x,y,z;
	x = RT[0][0]*xb + RT[0][1]*yb + RT[0][2]*zb;
	y = RT[1][0]*xb + RT[1][1]*yb + RT[1][2]*zb;
	z = RT[2][0]*xb + RT[2][1]*yb + RT[2][2]*zb;
	
	*xn=x*cosd(yaw)-y*sind(yaw);
	*yn=x*sind(yaw)+y*cosd(yaw);
	*zn=z;
}

//������ϵת�����ֲ�
char use_cor_trans=1;
void force_n_to_b(VMC *in)
{
	END_POS force_n;
	force_n.x=in->force_n[Xr];
	force_n.y=in->force_n[Yr];
	force_n.z=in->force_n[Zr];
	//ת��������  Xǰ��  Y�෽  Z�Ϸ�
	in->force_b[Xr] = vmc_all.Rn_b[0][0]*force_n.x + vmc_all.Rn_b[0][1]*force_n.y + vmc_all.Rn_b[0][2]*force_n.z;
	in->force_b[Yr] = vmc_all.Rn_b[1][0]*force_n.x + vmc_all.Rn_b[1][1]*force_n.y + vmc_all.Rn_b[1][2]*force_n.z;
	in->force_b[Zr] = vmc_all.Rn_b[2][0]*force_n.x + vmc_all.Rn_b[2][1]*force_n.y + vmc_all.Rn_b[2][2]*force_n.z;

	//no trans
	if(!use_cor_trans){
		in->force_b[Xr] = in->force_n[Xr];
		in->force_b[Yr] = in->force_n[Yr];
		in->force_b[Zr] = in->force_n[Zr];
	}
}