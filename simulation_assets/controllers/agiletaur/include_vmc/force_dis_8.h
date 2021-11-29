/*
 * File: force_dis8.h
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 07-Oct-2019 16:39:38
 */

#ifndef FORCE_DIS8_H
#define FORCE_DIS8_H

 /* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "force_dis8_types.h"

/* Function Declarations */
extern void force_dis_8new(const double FALL[3], const double TALL[3], const double Fp
	[4], const double PC[3], double P1[3], double P2[3], double
	P3[3], double P4[3], const char G[4], const double ATT_NOW[3],
	double Div_Roll, const char MODE[7], const char CAL_MODE[4],
	double F1c[3], double F2c[3], double F3c[3], double F4c[3]);
extern void force_dis_8_initialize(void);
extern void force_dis_8_terminate(void);
void force_dis_8new_initialize(void);
void rt_InitInfAndNaN();

#endif

/*
 * File trailer for force_dis8.h
 *
 * [EOF]
 */
