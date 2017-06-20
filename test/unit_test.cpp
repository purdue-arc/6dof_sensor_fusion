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

	ROS_DEBUG_STREAM("STATE JACO: " << ekf.computeStateTransitionJacobian(ekf.state, 0.1));
	ROS_DEBUG_STREAM("IMU JACO: " << ekf.computeIMUMeasurementJacobian(ekf.state));
}


