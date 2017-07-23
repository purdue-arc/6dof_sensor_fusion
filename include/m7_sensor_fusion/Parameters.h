/*
 * Parameters.h
 *
 *  Created on: Jun 19, 2017
 *      Author: pauvsi
 */

#ifndef M7_SENSOR_FUSION_INCLUDE_M7_SENSOR_FUSION_PARAMETERS_H_
#define M7_SENSOR_FUSION_INCLUDE_M7_SENSOR_FUSION_PARAMETERS_H_

#define SUPER_DEBUG true

#define MAXIMUM_OLD_STATES 1000

#define BASE_FRAME "base_link"
#define IMU_FRAME "imu_link"
#define DIPA_CAM_FRAME "bottom_camera"
#define SONAR_FRAME "sonar"

#define REMOVE_SIMILAR_MEASUREMENTS false

#define MANTIS_TOPIC "mantis/pose_estimate"
#define IMU_TOPIC "imu/measurement"
#define DIPA_TOPIC "dipa/twist_estimate"
#define SONAR_TOPIC "sonar/scan"

#define POSE_PUB_TOPIC "state/pose"
#define TWIST_PUB_TOPIC "state/twist"
#define ACCEL_PUB_TOPIC "state/accel"

// rate in hz which the state will be published at
#define STATE_PUBLISH_RATE 100

#define STATE_VECTOR_SIZE 15
#define MAX_ROWS 19

#define INITIAL_POS_SIGMA 10000
#define INITIAL_VEL_SIGMA 10
#define INITIAL_ACCEL_SIGMA 10
#define INITIAL_THETA_SIGMA 10
#define INITIAL_OMEGA_SIGMA 10

#define POS_PROCESS_SIGMA 1.0
#define VEL_PROCESS_SIGMA 2.0
#define ACCEL_PROCESS_SIGMA 3.0
#define THETA_PROCESS_SIGMA 0.5
#define OMEGA_PROCESS_SIGMA 0.02

#define POSE_PREDICT_SIGMA 3.0
#define TWIST_PREDICT_SIGMA 0.5

#define SONAR_DIST_SIG_MULTIPLIER 0.01
//always add this
#define CONSTANT_SONAR_SIG 0.03

#define IMU_ANGLE_SIGMA_ACCEL_MULTIPLIER 1.0

#define USE_SIM_BIASES true

#if !USE_SIM_BIASES
actual baises
#define AZ_BIAS 0.45
#define AY_BIAS -0.1
#define AX_BIAS 0.1
//-1.7 d/s
#define WX_BIAS -0.0296705973
// -1.4 d/s
#define WY_BIAS -0.0244346095
// 1.7 d/s
#define WZ_BIAS 0.0296706
#else
//sim biases
#define AX_BIAS 0.41
#define AY_BIAS 0.05
#define AZ_BIAS -0.03
#define WX_BIAS -0.12
#define WY_BIAS -0.031
#define WZ_BIAS -0.0304
#endif

#define G 9.815
#define PI 3.14159265359



#endif /* M7_SENSOR_FUSION_INCLUDE_M7_SENSOR_FUSION_PARAMETERS_H_ */
