/*
 * Types.h
 *
 *  Created on: Jun 19, 2017
 *      Author: pauvsi
 */

#ifndef M7_SENSOR_FUSION_INCLUDE_M7_SENSOR_FUSION_TYPES_H_
#define M7_SENSOR_FUSION_INCLUDE_M7_SENSOR_FUSION_TYPES_H_

struct State {
	Eigen::Matrix<double, STATE_VECTOR_SIZE, 1> vec; //x, y, z, dx, dy, dz, ax, ay, az, q0, q1, q2, q3, wx, wy, wz
	Eigen::Matrix<double, STATE_VECTOR_SIZE, STATE_VECTOR_SIZE> Sigma;
	ros::Time t;

	State() {
		vec(0) = 0.0;
		vec(1) = 0.0;
		vec(2) = 0.0;
		vec(3) = 0.0;
		vec(4) = 0.0;
		vec(5) = 0.0;
		vec(6) = 0.0;
		vec(7) = 0.0;
		vec(8) = 0.0;
		vec(9) = 1.0; // q0 = 1.0
		vec(10) = 0.0;
		vec(11) = 0.0;
		vec(12) = 0.0;
		vec(13) = 0.0;
		vec(14) = 0.0;
		vec(15) = 0.0;

		Sigma = Eigen::MatrixXd::Identity(16, 16);
		Sigma(0, 0) = INITIAL_POS_SIGMA;
		Sigma(1, 1) = INITIAL_POS_SIGMA;
		Sigma(2, 2) = INITIAL_POS_SIGMA;

		Sigma(3, 3) = INITIAL_VEL_SIGMA;
		Sigma(4, 4) = INITIAL_VEL_SIGMA;
		Sigma(5, 5) = INITIAL_VEL_SIGMA;

		Sigma(6, 6) = INITIAL_ACCEL_SIGMA;
		Sigma(7, 7) = INITIAL_ACCEL_SIGMA;
		Sigma(8, 8) = INITIAL_ACCEL_SIGMA;

		Sigma(9, 9) = INITIAL_QUAT_SIGMA;
		Sigma(10, 10) = INITIAL_QUAT_SIGMA;
		Sigma(11, 11) = INITIAL_QUAT_SIGMA;
		Sigma(12, 12) = INITIAL_QUAT_SIGMA;

		Sigma(13, 13) = INITIAL_OMEGA_SIGMA;
		Sigma(14, 14) = INITIAL_OMEGA_SIGMA;
		Sigma(15, 15) = INITIAL_OMEGA_SIGMA;

		t = ros::Time(0);
	}

	double x() {
		return vec(0);
	}
	double y() {
		return vec(1);
	}
	double z() {
		return vec(2);
	}
	double dx() {
		return vec(3);
	}
	double dy() {
		return vec(4);
	}
	double dz() {
		return vec(5);
	}
	double ax() {
		return vec(6);
	}
	double ay() {
		return vec(7);
	}
	double az() {
		return vec(8);
	}
	double q0() {
		return vec(9);
	}
	double q1() {
		return vec(10);
	}
	double q2() {
		return vec(11);
	}
	double q3() {
		return vec(12);
	}
	double wx() {
		return vec(13);
	}
	double wy() {
		return vec(14);
	}
	double wz() {
		return vec(15);
	}

	Eigen::Vector3d getPos() {
		return Eigen::Vector3d(x(), y(), z());
	}

	Eigen::Vector3d getVel() {
		return Eigen::Vector3d(dx(), dy(), dz());
	}

	Eigen::Vector3d getAccel() {
		return Eigen::Vector3d(ax(), ay(), az());
	}

	Eigen::Vector3d getOmega() {
		return Eigen::Vector3d(wx(), wy(), wz());
	}

	Eigen::Quaterniond getQuaternion() {
		return Eigen::Quaterniond(q0(), q1(), q2(), q3());
	}

	void setPos(Eigen::Vector3d in) {
		vec(0) = in(0);
		vec(1) = in(1);
		vec(2) = in(2);
	}

	void setVel(Eigen::Vector3d in) {
		vec(3) = in(0);
		vec(4) = in(1);
		vec(5) = in(2);
	}

	void setAccel(Eigen::Vector3d in) {
		vec(6) = in(0);
		vec(7) = in(1);
		vec(8) = in(2);
	}

	void setOmega(Eigen::Vector3d in) {
		vec(13) = in(0);
		vec(14) = in(1);
		vec(15) = in(2);
	}

	void setQuat(Eigen::Quaterniond in) {
		vec(9) = in.w();
		vec(10) = in.x();
		vec(11) = in.y();
		vec(12) = in.z();
	}

};

struct IMUMeasurement {
	Eigen::Matrix<double, 6, 1> z; // ax, ay, az, wx, wy, wz
	Eigen::Matrix<double, 6, 6> Sigma;
	ros::Time t;
	Eigen::Matrix<double, 6, STATE_VECTOR_SIZE> H;
};

struct PoseMeasurement {
	Eigen::Matrix<double, 7, 1> z; // x, y, z, q0, q1, q2, q3
	Eigen::Matrix<double, 7, 7> Sigma;
	ros::Time t;
	Eigen::Matrix<double, 7, STATE_VECTOR_SIZE> H;
};

struct TwistMeasurement {
	Eigen::Matrix<double, 6, 1> z; // dx, dy, dz, wx, wy, wz
	Eigen::Matrix<double, 6, 6> Sigma;
	Eigen::Matrix<double, 6, STATE_VECTOR_SIZE> H;
	ros::Time t;
};

struct Measurement {
private:
	enum Type {
		IMU, POSE, TWIST, NONE
	};
	Type type;
	IMUMeasurement imu;
	PoseMeasurement pose;
	TwistMeasurement twist;

public:

	Measurement(){
		type = NONE;
	}

	Measurement(IMUMeasurement z) {
		imu = z;
		type = IMU;
	}

	Measurement(PoseMeasurement z) {
		pose = z;
		type = POSE;
	}

	Measurement(TwistMeasurement z) {
		twist = z;
		type = TWIST;
	}

	Type getType(){
		return type;
	}

	ros::Time getTime() {
		switch (type) {
		case IMU:
			return imu.t;
			break;
		case POSE:
			return pose.t;
			break;
		case TWIST:
			return twist.t;
			break;
		default:
			ROS_FATAL("measurement type invalid");
			return ros::Time(0);
			break;
		}
	}

	Eigen::MatrixXd getSigma() {
		switch (type) {
		case IMU:
			return imu.Sigma;
			break;
		case POSE:
			return pose.Sigma;
			break;
		case TWIST:
			return twist.Sigma;
			break;
		default:
			ROS_FATAL("measurement type invalid");
			return Eigen::MatrixXd();
			break;
		}
	}

	Eigen::MatrixXd getZ() {
		switch (type) {
		case IMU:
			return imu.z;
			break;
		case POSE:
			return pose.z;
			break;
		case TWIST:
			return twist.z;
			break;
		default:
			ROS_FATAL("measurement type invalid");
			return Eigen::MatrixXd();
			break;
		}
	}

	Eigen::MatrixXd getH() {
		switch (type) {
		case IMU:
			return imu.H;
			break;
		case POSE:
			return pose.H;
			break;
		case TWIST:
			return twist.H;
			break;
		default:
			ROS_FATAL("measurement type invalid");
			return Eigen::MatrixXd();
			break;
		}
	}

};

struct MeasurementCombination {
	Eigen::MatrixXd z;
	Eigen::MatrixXd Sigma;
	Eigen::MatrixXd H;
	ros::Time t;

	/*
	 * all measurements must be predicted or set to the same time as prediction
	 */
	MeasurementCombination(std::vector<Measurement> meas, State prediction){
		int z_rows = 0;

		for(auto e : meas)
		{
			ROS_ASSERT(e.getTime() == prediction.t);
			z_rows += e.getZ().rows();
		}

		z.resize(z_rows, 1);
		H.resize(z_rows, STATE_VECTOR_SIZE);
		Sigma = Eigen::MatrixXd::Zero(z_rows, z_rows);

		int i = 0;
		for(auto e : meas)
		{
			z.block(i, 0, e.getZ().rows(), 1) = e.getZ();
			H.block(i, 0, e.getH().rows(), STATE_VECTOR_SIZE) = e.getH();
			Sigma.block(i, i, e.getSigma().rows(), e.getSigma().rows()) = e.getSigma();
			i += e.getZ().rows();
		}

		t = prediction.t;
	}
};

#endif /* M7_SENSOR_FUSION_INCLUDE_M7_SENSOR_FUSION_TYPES_H_ */
