/*
 * Jacobian.cpp
 *
 *  Created on: Jun 20, 2017
 *      Author: pauvsi
 */

#include "EKF.h"

Eigen::Matrix<double, 16, 16> EKF::computeStateTransitionJacobian(State state, double dt)
{

	double x = state.x();
	double y = state.x();
	double z = state.x();
	double dx = state.dx();
	double dy = state.dy();
	double dz = state.dz();
	double ax = state.ax();
	double ay = state.ay();
	double az = state.az();
	double q0 = state.q0();
	double q1 = state.q1();
	double q2 = state.q2();
	double q3 = state.q3();
	double wx = state.wz();
	double wy = state.wy();
	double wz = state.wz();

	double A0[16][16] = {0};

	//generated with matlab
	double t2 = dt*dt;
	double t3 = t2*(1.0/2.0);
	double t4 = wx*wx;
	double t5 = t2*t4;
	double 	t6 = wy*wy;
	double 	t7 = t2*t6;
	double 	t8 = wz*wz;
	double 	t9 = t2*t8;
	double 	t10 = t5+t7+t9; // omega mag
	double 	t11 = sqrt(t10);
	double 	t12 = t11*(1.0/2.0);
	double 	t13 = sin(t12);
	double 	t14 = 1.0/sqrt(t10);
	if(t10 == 0){
		t14 = 0;
	}
	double 	t15 = cos(t12);
	double 	t16 = 1.0/t10;
	if(t10 == 0){
		t16 = 0;
	}
	double 	t17 = 1.0/pow(t10,3.0/2.0);
	if(t10 == 0){
		t17 = 0;
	}
	double 	t18 = q2*t2*t13*t17*wy*wz;
	double 	t19 = q3*t2*t13*t17*wy*wz;
	double 	t20 = t13*t14*wz;
	double 	t21 = t13*t14*wx;
	double 	t22 = q0*t2*t15*t16*wx*wy*(1.0/2.0);
	double 	t23 = q1*t2*t13*t17*wx*wz;
	double 	t24 = q0*t13*t14;
	double 	t25 = q3*t2*t13*t17*wx*wy;
	double 	t26 = q3*t2*t13*t17*wx*wz;
	double 	t27 = t13*t14*wy;
	double 	t28 = q2*t13*t14;
	double 	t29 = q0*t2*t15*t16*wx*wz*(1.0/2.0);
	double 	t30 = q1*t2*t13*t17*wx*wy;
	double 	t31 = q0*t2*t15*t16*wy*wz*(1.0/2.0);
	double 	t32 = q2*t2*t13*t17*wx*wy;
	double 	t33 = q2*t2*t15*t16*wx*wz*(1.0/2.0);
	double 	t34 = q1*t2*t13*t17*wy*wz;
	A0[0][0] = 1.0;
	A0[0][3] = dt;
	A0[0][6] = t3;
	A0[1][1] = 1.0;
	A0[1][4] = dt;
	A0[1][7] = t3;
	A0[2][2] = 1.0;
	A0[2][5] = dt;
	A0[2][8] = t3;
	A0[3][3] = 1.0;
	A0[3][6] = dt;
	A0[4][4] = 1.0;
	A0[4][7] = dt;
	A0[5][5] = 1.0;
	A0[5][8] = dt;
	A0[6][6] = 1.0;
	A0[7][7] = 1.0;
	A0[8][8] = 1.0;
	A0[9][9] = t15;
	A0[9][10] = -t13*t14*wx;
	A0[9][11] = -t13*t14*wy;
	A0[9][12] = -t13*t14*wz;
	A0[9][13] = t26+t32-q1*t13*t14+q1*t2*t4*t13*t17-q1*t2*t4*t15*t16*(1.0/2.0)-q0*t2*t13*t14*wx*(1.0/2.0)-q2*t2*t15*t16*wx*wy*(1.0/2.0)-q3*t2*t15*t16*wx*wz*(1.0/2.0);
	A0[9][14] = t19+t30-q2*t13*t14+q2*t2*t6*t13*t17-q2*t2*t6*t15*t16*(1.0/2.0)-q0*t2*t13*t14*wy*(1.0/2.0)-q1*t2*t15*t16*wx*wy*(1.0/2.0)-q3*t2*t15*t16*wy*wz*(1.0/2.0);
	A0[9][15] = t18+t23-q3*t13*t14+q3*t2*t8*t13*t17-q3*t2*t8*t15*t16*(1.0/2.0)-q0*t2*t13*t14*wz*(1.0/2.0)-q1*t2*t15*t16*wx*wz*(1.0/2.0)-q2*t2*t15*t16*wy*wz*(1.0/2.0);
	A0[10][9] = t21;
	A0[10][10] = t15;
	A0[10][11] = t20;
	A0[10][12] = -t13*t14*wy;
	A0[10][13] = t24+t25+t33-q0*t2*t4*t13*t17+q0*t2*t4*t15*t16*(1.0/2.0)-q1*t2*t13*t14*wx*(1.0/2.0)-q3*t2*t15*t16*wx*wy*(1.0/2.0)-q2*t2*t13*t17*wx*wz;
	A0[10][14] = -t18+t22-q3*t13*t14+q3*t2*t6*t13*t17-q3*t2*t6*t15*t16*(1.0/2.0)-q1*t2*t13*t14*wy*(1.0/2.0)-q0*t2*t13*t17*wx*wy+q2*t2*t15*t16*wy*wz*(1.0/2.0);
	A0[10][15] = t19+t28+t29-q2*t2*t8*t13*t17+q2*t2*t8*t15*t16*(1.0/2.0)-q1*t2*t13*t14*wz*(1.0/2.0)-q0*t2*t13*t17*wx*wz-q3*t2*t15*t16*wy*wz*(1.0/2.0);
	A0[11][9] = t27;
	A0[11][10] = -t20;
	A0[11][11] = t15;
	A0[11][12] = t21;
	A0[11][13] = t22+t23+q3*t13*t14-q3*t2*t4*t13*t17+q3*t2*t4*t15*t16*(1.0/2.0)-q2*t2*t13*t14*wx*(1.0/2.0)-q0*t2*t13*t17*wx*wy-q1*t2*t15*t16*wx*wz*(1.0/2.0);
	A0[11][14] = t24-t25+t34-q0*t2*t6*t13*t17+q0*t2*t6*t15*t16*(1.0/2.0)-q2*t2*t13*t14*wy*(1.0/2.0)+q3*t2*t15*t16*wx*wy*(1.0/2.0)-q1*t2*t15*t16*wy*wz*(1.0/2.0);
	A0[11][15] = -t26+t31-q1*t13*t14+q1*t2*t8*t13*t17-q1*t2*t8*t15*t16*(1.0/2.0)-q2*t2*t13*t14*wz*(1.0/2.0)+q3*t2*t15*t16*wx*wz*(1.0/2.0)-q0*t2*t13*t17*wy*wz;
	A0[12][9] = t20;
	A0[12][10] = t27;
	A0[12][11] = -t21;
	A0[12][12] = t15;
	A0[12][13] = -t28+t29-t30+q2*t2*t4*t13*t17-q2*t2*t4*t15*t16*(1.0/2.0)-q3*t2*t13*t14*wx*(1.0/2.0)+q1*t2*t15*t16*wx*wy*(1.0/2.0)-q0*t2*t13*t17*wx*wz;
	A0[12][14] = t31+t32+q1*t13*t14-q1*t2*t6*t13*t17+q1*t2*t6*t15*t16*(1.0/2.0)-q3*t2*t13*t14*wy*(1.0/2.0)-q2*t2*t15*t16*wx*wy*(1.0/2.0)-q0*t2*t13*t17*wy*wz;
	A0[12][15] = t24-t33-t34-q0*t2*t8*t13*t17+q0*t2*t8*t15*t16*(1.0/2.0)-q3*t2*t13*t14*wz*(1.0/2.0)+q2*t2*t13*t17*wx*wz+q1*t2*t15*t16*wy*wz*(1.0/2.0);
	A0[13][13] = 1.0;
	A0[14][14] = 1.0;
	A0[15][15] = 1.0;


	Eigen::Matrix<double, 16, 16> F;

	for(int i = 0; i < 16; i++)
	{
		for(int j = 0; j < 16; j++)
		{
			F(i, j) = A0[i][j];
		}
	}

	return F;

}


Eigen::Matrix<double, 6, 16> EKF::computeIMUMeasurementJacobian(State est)
{

	double ax = state.ax();
	double ay = state.ay();
	double az = state.az();
	double q0 = state.q0();
	double q1 = state.q1();
	double q2 = state.q2();
	double q3 = state.q3();

	double t2 = G+az;
	double t3 = q0*q3*2.0;
	double t4 = q1*q2*2.0;
	double t5 = q3*q3;
	double t6 = q1*t2*2.0;
	double t7 = q3*t2*2.0;
	double t8 = q1*q3*2.0;
	double t9 = q0*q1*2.0;
	double t10 = q2*q3*2.0;
	double t11 = q1*q1;
	double t12 = q2*q2;
	double t13 = ax*q2*2.0;
	double t14 = ay*q1*2.0;
	double t15 = ay*q0*2.0;
	double t16 = ay*q3*2.0;
	double t17 = ax*q1*2.0;
	double t18 = ay*q2*2.0;

	double A0[6][16] = {0};

	A0[0][6] = t5*-2.0-t12*2.0+1.0;
	A0[0][7] = t3+t4;
	A0[0][8] = t8-q0*q2*2.0;
	A0[0][9] = t16-q2*t2*2.0;
	A0[0][10] = t7+t18;
	A0[0][11] = t14-ax*q2*4.0-q0*t2*2.0;
	A0[0][12] = t6+t15-ax*q3*4.0;
	A0[1][6] = -t3+t4;
	A0[1][7] = t5*-2.0-t11*2.0+1.0;
	A0[1][8] = t9+t10;
	A0[1][9] = t6-ax*q3*2.0;
	A0[1][10] = t13-ay*q1*4.0+q0*t2*2.0;
	A0[1][11] = t7+t17;
	A0[1][12] = ax*q0*-2.0-ay*q3*4.0+q2*t2*2.0;
	A0[2][6] = t8+q0*q2*2.0;
	A0[2][7] = -t9+t10;
	A0[2][8] = t11*-2.0-t12*2.0+1.0;
	A0[2][9] = t13-t14;
	A0[2][10] = -t15+ax*q3*2.0-q1*t2*4.0;
	A0[2][11] = t16+ax*q0*2.0-q2*t2*4.0;
	A0[2][12] = t17+t18;
	A0[3][13] = 1.0;
	A0[4][14] = 1.0;
	A0[5][15] = 1.0;


	Eigen::Matrix<double, 6, 16> H;

	for(int i = 0; i < 6; i++)
	{
		for(int j = 0; j < 16; j++)
		{
			H(i, j) = A0[i][j];
		}
	}

	return H;

}


