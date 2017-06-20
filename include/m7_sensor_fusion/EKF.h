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

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>

#include "Parameters.h"
#include "Types.h"

class EKF {
public:

	std::vector<State> old_states; // used for interpolating old measurements forward to be used

	State state;

	tf::Transform base2imu; // can transform imu measurements into the base coordinate frame

	tf::TransformListener tf_listener; // listens for transforms

	EKF(bool test = false);
	virtual ~EKF();

	void imu_callback(sensor_msgs::ImuConstPtr& msg);
	void mantis_callback(geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
	void dipa_callback(geometry_msgs::TwistWithCovarianceStampedConstPtr& msg);

	State process(State prior, ros::Time t);

	State update(State prior, IMUMeasurement measurement);

	Eigen::Matrix<double, 16, 16> computeStateTransitionJacobian(State est, double dt);

	Eigen::Matrix<double, 6, 16> computeIMUMeasurementJacobian(State est);


};

#endif /* M7_SENSOR_FUSION_INCLUDE_M7_SENSOR_FUSION_EKF_H_ */
