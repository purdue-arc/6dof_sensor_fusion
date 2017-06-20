/*
 * Types.h
 *
 *  Created on: Jun 19, 2017
 *      Author: pauvsi
 */

#ifndef M7_SENSOR_FUSION_INCLUDE_M7_SENSOR_FUSION_TYPES_H_
#define M7_SENSOR_FUSION_INCLUDE_M7_SENSOR_FUSION_TYPES_H_


struct State{
	Eigen::Matrix<double, 16, 1> x; //x, y, z, dx, dy, dz, ax, ay, az, q0, q1, q2, q3, wx, wy, wz
	Eigen::Matrix<double, 16, 16> Sigma;
	ros::Time t;
};

struct IMUMeasurement{
	Eigen::Matrix<double, 6, 1> z;
	Eigen::Matrix<double, 6, 6> Sigma;
	ros::Time t;
};


#endif /* M7_SENSOR_FUSION_INCLUDE_M7_SENSOR_FUSION_TYPES_H_ */
