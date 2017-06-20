/*
 * sensor_fusion.cpp
 *
 *  Created on: Jun 19, 2017
 *      Author: pauvsi
 */

#include <ros/ros.h>

#include <m7_sensor_fusion/EKF.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "sensor_fusion");

	EKF ekf;

	return 0;
}
