/*
 * unit_test.cpp
 *
 *  Created on: Jun 20, 2017
 *      Author: pauvsi
 */

#include <ros/ros.h>

#include <m7_sensor_fusion/EKF.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "sensor_fusion");

	EKF ekf(true);

	ROS_DEBUG_STREAM("STATE JACO1: " << ekf.computeStateTransitionF(0.1));

	/*ekf.state.setOmega(Eigen::Vector3d(1/sqrt(3), 1/sqrt(3), 1/sqrt(3)));
	ROS_DEBUG_STREAM("STATE JACO2: " << ekf.computeStateTransitionF(0.1));
	ekf.state.setOmega(Eigen::Vector3d(0, 0, 0));

	ekf.state.setAccel(Eigen::Vector3d(0, 0, 1));
	ROS_DEBUG_STREAM("STATE JACO3: " << ekf.computeStateTransitionJacobian(ekf.state, 0.1));

	ekf.state.setQuat(Eigen::Quaterniond(1/sqrt(2), 1/sqrt(2), 0, 0));
	ROS_DEBUG_STREAM("STATE JACO4: " << ekf.computeStateTransitionJacobian(ekf.state, 0.1));

	ekf.state.setQuat(Eigen::Quaterniond(1/sqrt(2), 0, 0, 1/sqrt(2)));
	ROS_DEBUG_STREAM("STATE JACO5: " << ekf.computeStateTransitionJacobian(ekf.state, 0.1));


	ROS_DEBUG_STREAM("IMU JACO1: " << ekf.computeIMUMeasurementJacobian(ekf.state));*/

	ekf.state.setTheta(Eigen::Vector3d(0, 0, 0));
	ekf.state.setAccel(Eigen::Vector3d(0, 0, 0));

	IMUMeasurement z_imu;
	z_imu.Sigma = Eigen::MatrixXd::Identity(5, 5) * 0.01;
	z_imu.z = Eigen::VectorXd::Zero(5, 1);
	z_imu.z << 0, 0, 1, 1, 1;
	z_imu.H = Eigen::MatrixXd::Identity(5, STATE_VECTOR_SIZE);
	z_imu.t = ekf.state.t;

	ROS_DEBUG("made zimu");

	PoseMeasurement z_pose;
	z_pose.Sigma = Eigen::MatrixXd::Identity(6, 6) * 0.02;
	z_pose.z = Eigen::VectorXd::Zero(6, 1);
	z_pose.z << 2, 2, 2, 2, 2, 2;
	z_pose.H = Eigen::MatrixXd::Identity(6, STATE_VECTOR_SIZE);
	z_pose.t = ekf.state.t;

	ROS_DEBUG("made zpose");

	std::deque<Measurement> meas;
	meas.push_back(Measurement(z_pose));
	meas.push_back(Measurement(z_imu));

	ROS_DEBUG("pushed");

	MeasurementCombination combo = MeasurementCombination(meas, ekf.state);

	ROS_DEBUG_STREAM(combo.z);
	ROS_DEBUG_STREAM(combo.Sigma);
	ROS_DEBUG_STREAM(combo.H);
}


