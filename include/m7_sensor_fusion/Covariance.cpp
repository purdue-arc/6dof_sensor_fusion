/*
 * Covariance.cpp
 *
 *  Created on: Jun 20, 2017
 *      Author: pauvsi
 */

#include "EKF.h"

Eigen::Matrix<double, STATE_VECTOR_SIZE, STATE_VECTOR_SIZE> EKF::computeStateProcessError(double dt)
{
	Eigen::Matrix<double, STATE_VECTOR_SIZE, STATE_VECTOR_SIZE> Q = Eigen::MatrixXd::Identity(STATE_VECTOR_SIZE, STATE_VECTOR_SIZE);

	Q(0, 0) = POS_PROCESS_SIGMA * dt;
	Q(1, 1) = POS_PROCESS_SIGMA * dt;
	Q(2, 2) = POS_PROCESS_SIGMA * dt;
	Q(3, 3) = VEL_PROCESS_SIGMA * dt;
	Q(4, 4) = VEL_PROCESS_SIGMA * dt;
	Q(5, 5) = VEL_PROCESS_SIGMA * dt;
	Q(6, 6) = ACCEL_PROCESS_SIGMA * dt;
	Q(7, 7) = ACCEL_PROCESS_SIGMA * dt;
	Q(8, 8) = ACCEL_PROCESS_SIGMA * dt;
	Q(9, 9) = THETA_PROCESS_SIGMA * dt;
	Q(10, 10) = THETA_PROCESS_SIGMA * dt;
	Q(11, 11) = THETA_PROCESS_SIGMA * dt;
	Q(12, 12) = OMEGA_PROCESS_SIGMA * dt;
	Q(13, 13) = OMEGA_PROCESS_SIGMA * dt;
	Q(14, 14) = OMEGA_PROCESS_SIGMA * dt;

	return Q;

}


