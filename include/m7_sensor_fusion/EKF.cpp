/*
 * EKF.cpp
 *
 *  Created on: Jun 19, 2017
 *      Author: pauvsi
 */

#include "EKF.h"

EKF::EKF() {
	ros::NodeHandle nh;

	// get the transform between the imu and base
	ROS_INFO_STREAM("WAITING FOR TANSFORM FROM " << BASE_FRAME << " TO " << IMU_FRAME);
	if(tf_listener.waitForTransform(BASE_FRAME, IMU_FRAME, ros::Time(0), ros::Duration(2))){
		tf::StampedTransform b2i;
		try {
			tf_listener.lookupTransform(BASE_FRAME, IMU_FRAME,
					ros::Time(0), b2i);
		} catch (tf::TransformException& e) {
			ROS_WARN_STREAM(e.what());
		}
		base2imu = tf::Transform(b2i);
		ROS_INFO("GOT TRANSFORM");
	}
	else
	{
		ROS_FATAL("COULD NOT GET TRANSFORM");
		ros::shutdown();
		return;
	}

	ros::spin();
}

EKF::~EKF() {
	// TODO Auto-generated destructor stub
}

void EKF::imu_callback(sensor_msgs::ImuConstPtr& msg)
{

}

void EKF::mantis_callback(geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{

}

void EKF::dipa_callback(geometry_msgs::TwistWithCovarianceStampedConstPtr& msg)
{

}

State EKF::process(State prior)
{

}

State EKF::update(State prior, IMUMeasurement measurement)
{

}

Eigen::Matrix<double, 16, 16> EKF::computeStateTransitionMatrix(double dt)
{

}
