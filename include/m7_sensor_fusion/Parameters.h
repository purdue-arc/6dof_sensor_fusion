/*
 * Parameters.h
 *
 *  Created on: Jun 19, 2017
 *      Author: pauvsi
 */

#ifndef M7_SENSOR_FUSION_INCLUDE_M7_SENSOR_FUSION_PARAMETERS_H_
#define M7_SENSOR_FUSION_INCLUDE_M7_SENSOR_FUSION_PARAMETERS_H_

#define MAXIMUM_OLD_STATES 1000

#define BASE_FRAME "base_link"
#define IMU_FRAME "imu_frame"

#define MANTIS_TOPIC "mantis/pose_estimate"
#define IMU_TOPIC "imu/measurements"
#define DIPA_TOPIC "dipa/twist_estimate"

// rate in hz which the state will be published at
#define STATE_PUBLISH_RATE 100

#define STATE_VECTOR_SIZE 16
#define MAX_ROWS 19

#define INITIAL_POS_SIGMA 10000
#define INITIAL_VEL_SIGMA 10
#define INITIAL_ACCEL_SIGMA 10
#define INITIAL_QUAT_SIGMA 10
#define INITIAL_OMEGA_SIGMA 10

#define POS_PROCESS_SIGMA 0.1
#define VEL_PROCESS_SIGMA 0.05
#define ACCEL_PROCESS_SIGMA 0.01
#define QUAT_PROCESS_SIGMA 0.1
#define OMEGA_PROCESS_SIGMA 0.01


#define G 9.815



#endif /* M7_SENSOR_FUSION_INCLUDE_M7_SENSOR_FUSION_PARAMETERS_H_ */
