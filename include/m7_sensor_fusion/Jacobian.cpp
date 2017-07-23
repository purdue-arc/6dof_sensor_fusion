/*
 * Jacobian.cpp
 *
 *  Created on: Jun 20, 2017
 *      Author: pauvsi
 */

#include "EKF.h"

Eigen::Matrix<double, STATE_VECTOR_SIZE, STATE_VECTOR_SIZE> EKF::computeStateTransitionF(
		double dt) {
	Eigen::Matrix<double, STATE_VECTOR_SIZE, STATE_VECTOR_SIZE> F;
	F.setIdentity();

	F(0, 3) = dt;
	F(1, 4) = dt;
	F(2, 5) = dt;
	F(0, 6) = 0.5 * dt * dt;
	F(1, 7) = 0.5 * dt * dt;
	F(2, 8) = 0.5 * dt * dt;

	F(3, 6) = dt;
	F(4, 7) = dt;
	F(5, 8) = dt;

	F(9, 12) = dt;
	F(10, 13) = dt;
	F(11, 14) = dt;

	//ROS_DEBUG_STREAM("F: " << F);

	return F;
}
/*Eigen::Matrix<double, STATE_VECTOR_SIZE, STATE_VECTOR_SIZE> EKF::computeStateTransitionJacobian(State state, double dt)
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
 double  t5 = wy*wy;
 double 	  t6 = wz*wz;
 double 	  t7 = t4+t5+t6;
 double 	  t8 = sqrt(t7);
 double	  t9 = dt*t8*(1.0/2.0);
 double 	  t10 = sin(t9);
 double   t11 = 1.0/sqrt(t7);
 if(t7 == 0)
 {
 t11 = 0;
 }
 double 	  t12 = 1.0/pow(t7,3.0/2.0);
 if(t7 == 0)
 {
 t12 = 0;
 }
 double 	  t13 = cos(t9);
 double 	  t14 = 1.0/t7;
 if(t7 == 0)
 {
 t14 = 0;
 }
 double 	  t15 = q2*t10*t12*wy*wz;
 double 	  t16 = q3*t10*t12*wy*wz;
 double 	  t17 = t10*t11*wz;
 double 	  t18 = t10*t11*wx;
 double 	  t19 = q1*t10*t12*wx*wz;
 double 	  t20 = dt*q0*t13*t14*wx*wy*(1.0/2.0);
 double 	  t21 = q0*t10*t11;
 double 	  t22 = q3*t10*t12*wx*wy;
 double 	  t23 = q3*t10*t12*wx*wz;
 double 	  t24 = t10*t11*wy;
 double 	  t25 = q2*t10*t11;
 double 	  t26 = q1*t10*t12*wx*wy;
 double 	  t27 = dt*q0*t13*t14*wx*wz*(1.0/2.0);
 double 	  t28 = q2*t10*t12*wx*wy;
 double 	  t29 = dt*q0*t13*t14*wy*wz*(1.0/2.0);
 double 	  t30 = q1*t10*t12*wy*wz;
 double 	  t31 = dt*q2*t13*t14*wx*wz*(1.0/2.0);
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
 A0[9][9] = t13;
 A0[9][10] = -t10*t11*wx;
 A0[9][11] = -t10*t11*wy;
 A0[9][12] = -t10*t11*wz;
 A0[9][13] = t23+t28-q1*t10*t11+q1*t4*t10*t12-dt*q1*t4*t13*t14*(1.0/2.0)-dt*q0*t10*t11*wx*(1.0/2.0)-dt*q2*t13*t14*wx*wy*(1.0/2.0)-dt*q3*t13*t14*wx*wz*(1.0/2.0);
 A0[9][14] = t16+t26-q2*t10*t11+q2*t5*t10*t12-dt*q2*t5*t13*t14*(1.0/2.0)-dt*q0*t10*t11*wy*(1.0/2.0)-dt*q1*t13*t14*wx*wy*(1.0/2.0)-dt*q3*t13*t14*wy*wz*(1.0/2.0);
 A0[9][15] = t15+t19-q3*t10*t11+q3*t6*t10*t12-dt*q3*t6*t13*t14*(1.0/2.0)-dt*q0*t10*t11*wz*(1.0/2.0)-dt*q1*t13*t14*wx*wz*(1.0/2.0)-dt*q2*t13*t14*wy*wz*(1.0/2.0);
 A0[10][9] = t18;
 A0[10][10] = t13;
 A0[10][11] = t17;
 A0[10][12] = -t10*t11*wy;
 A0[10][13] = t21+t22+t31-q0*t4*t10*t12+dt*q0*t4*t13*t14*(1.0/2.0)-dt*q1*t10*t11*wx*(1.0/2.0)-q2*t10*t12*wx*wz-dt*q3*t13*t14*wx*wy*(1.0/2.0);
 A0[10][14] = -t15+t20-q3*t10*t11+q3*t5*t10*t12-dt*q3*t5*t13*t14*(1.0/2.0)-dt*q1*t10*t11*wy*(1.0/2.0)-q0*t10*t12*wx*wy+dt*q2*t13*t14*wy*wz*(1.0/2.0);
 A0[10][15] = t16+t25+t27-q2*t6*t10*t12+dt*q2*t6*t13*t14*(1.0/2.0)-dt*q1*t10*t11*wz*(1.0/2.0)-q0*t10*t12*wx*wz-dt*q3*t13*t14*wy*wz*(1.0/2.0);
 A0[11][9] = t24;
 A0[11][10] = -t17;
 A0[11][11] = t13;
 A0[11][12] = t18;
 A0[11][13] = t19+t20+q3*t10*t11-q3*t4*t10*t12+dt*q3*t4*t13*t14*(1.0/2.0)-dt*q2*t10*t11*wx*(1.0/2.0)-q0*t10*t12*wx*wy-dt*q1*t13*t14*wx*wz*(1.0/2.0);
 A0[11][14] = t21-t22+t30-q0*t5*t10*t12+dt*q0*t5*t13*t14*(1.0/2.0)-dt*q2*t10*t11*wy*(1.0/2.0)+dt*q3*t13*t14*wx*wy*(1.0/2.0)-dt*q1*t13*t14*wy*wz*(1.0/2.0);
 A0[11][15] = -t23+t29-q1*t10*t11+q1*t6*t10*t12-dt*q1*t6*t13*t14*(1.0/2.0)-dt*q2*t10*t11*wz*(1.0/2.0)-q0*t10*t12*wy*wz+dt*q3*t13*t14*wx*wz*(1.0/2.0);
 A0[12][9] = t17;
 A0[12][10] = t24;
 A0[12][11] = -t18;
 A0[12][12] = t13;
 A0[12][13] = -t25-t26+t27+q2*t4*t10*t12-dt*q2*t4*t13*t14*(1.0/2.0)-dt*q3*t10*t11*wx*(1.0/2.0)-q0*t10*t12*wx*wz+dt*q1*t13*t14*wx*wy*(1.0/2.0);
 A0[12][14] = t28+t29+q1*t10*t11-q1*t5*t10*t12+dt*q1*t5*t13*t14*(1.0/2.0)-dt*q3*t10*t11*wy*(1.0/2.0)-q0*t10*t12*wy*wz-dt*q2*t13*t14*wx*wy*(1.0/2.0);
 A0[12][15] = t21-t30-t31-q0*t6*t10*t12+dt*q0*t6*t13*t14*(1.0/2.0)-dt*q3*t10*t11*wz*(1.0/2.0)+q2*t10*t12*wx*wz+dt*q1*t13*t14*wy*wz*(1.0/2.0);
 A0[13][13] = 1.0;
 A0[14][14] = 1.0;
 A0[15][15] = 1.0;



 Eigen::Matrix<double, STATE_VECTOR_SIZE, STATE_VECTOR_SIZE> F;

 for(int i = 0; i < 16; i++)
 {
 for(int j = 0; j < 16; j++)
 {
 F(i, j) = A0[i][j];
 }
 }

 return F;

 }*/

Eigen::Matrix<double, 6, STATE_VECTOR_SIZE> EKF::computePoseMeasurementH() {
	Eigen::Matrix<double, 6, STATE_VECTOR_SIZE> H;
	H.setZero();

	H(0, 0) = 1.0;
	H(1, 1) = 1.0;
	H(2, 2) = 1.0;
	H(3, 9) = 1.0;
	H(4, 10) = 1.0;
	H(5, 11) = 1.0;

	return H;
}

Eigen::Matrix<double, 5, STATE_VECTOR_SIZE> EKF::computeIMUMeasurementH() {
	Eigen::Matrix<double, 5, STATE_VECTOR_SIZE> H;
	H.setZero();

	H(0, 9) = 1.0; //roll
	H(1, 10) = 1.0; // pitch
	H(2, 12) = 1.0; // wx
	H(3, 13) = 1.0; // wx
	H(4, 14) = 1.0; // wx

	return H;
}

Eigen::Matrix<double, 6, STATE_VECTOR_SIZE> EKF::computeTwistMeasurementH()
{
	Eigen::Matrix<double, 6, STATE_VECTOR_SIZE> H;
	H.setZero();

	H(0, 3) = 1.0;
	H(1, 4) = 1.0;
	H(2, 5) = 1.0;
	H(3, 12) = 1.0;
	H(4, 13) = 1.0;
	H(5, 14) = 1.0;

	return H;
}

/*Eigen::Matrix<double, 6, STATE_VECTOR_SIZE> EKF::computeIMUMeasurementJacobian(State est)
 {

 double ax = state.ax();
 double ay = state.ay();
 double az = state.az();
 double q0 = state.q0();
 double q1 = state.q1();
 double q2 = state.q2();
 double q3 = state.q3();

 double gravity = -G;

 double t2 = gravity+az;
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


 Eigen::Matrix<double, 6, STATE_VECTOR_SIZE> H;

 for(int i = 0; i < 6; i++)
 {
 for(int j = 0; j < 16; j++)
 {
 H(i, j) = A0[i][j];
 }
 }

 return H;

 }*/

