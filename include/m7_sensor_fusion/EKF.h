/*
 * EKF.h
 *
 *  Created on: Jun 19, 2017
 *      Author: pauvsi
 */

#ifndef M7_SENSOR_FUSION_INCLUDE_M7_SENSOR_FUSION_EKF_H_
#define M7_SENSOR_FUSION_INCLUDE_M7_SENSOR_FUSION_EKF_H_

#include <ros/ros.h>
#include <eigen3/Eigen/Geometry>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class EKF {
public:
	EKF();
	virtual ~EKF();

	void imu_callback(sensor_msgs::ImuConstPtr& msg);
	void mantis_callback(geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
	void dipa_callback(geometry_msgs::TwistWithCovarianceStampedConstPtr& msg);
};

#endif /* M7_SENSOR_FUSION_INCLUDE_M7_SENSOR_FUSION_EKF_H_ */
