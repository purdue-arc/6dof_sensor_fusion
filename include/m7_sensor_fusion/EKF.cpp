/*
 * EKF.cpp
 *
 *  Created on: Jun 19, 2017
 *      Author: pauvsi
 */

#include "EKF.h"

EKF::EKF(bool test) {
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

	// set up subs
	ros::Subscriber imu_sub, mantis_sub, dipa_sub;

	imu_sub = nh.subscribe<sensor_msgs::Imu>(IMU_TOPIC, 100, &EKF::imu_callback, this);
	mantis_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(MANTIS_TOPIC, 2, &EKF::mantis_callback, this);
	dipa_sub = nh.subscribe<nav_msgs::Odometry>(DIPA_TOPIC, 10, &EKF::dipa_callback, this);

	// set up pubs
	ros::Publisher pose_pub, twist_pub, accel_pub;

	pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(POSE_PUB_TOPIC, 1);
	twist_pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>(TWIST_PUB_TOPIC, 1);
	accel_pub = nh.advertise<geometry_msgs::AccelWithCovarianceStamped>(ACCEL_PUB_TOPIC, 1);

	// run the filter
	if(!test)
	{
		ros::Rate loop_rate(STATE_PUBLISH_RATE);

		while(ros::ok())
		{
			ros::spinOnce();
			//TODO set the start time if ros::Time(0) = the state's time
			//TODO only start predicting and fusing when their is a odometry estimate
			State last_state = this->state;
			this->state = this->process(this->state, ros::Time::now());

			this->predictAllMeasurementsForward(this->state.t);

			MeasurementCombination mc = MeasurementCombination(this->measurements, this->state);

			//clear the measurement
			this->measurements.clear();

			this->state = this->update(this->state, mc);

			loop_rate.sleep(); // sleep until the next time we should predict and update
		}
	}
}

EKF::~EKF() {
	// TODO Auto-generated destructor stub
}

double EKF::constrainAngle(double angle)
{
	double new_angle = angle;
	bool pass = false;
	while(!pass)
	{
		if(new_angle < -PI)
		{
			new_angle += 2*PI;
		}
		else if(new_angle >= PI)
		{
			new_angle -= 2*PI;
		}
		else
		{
			pass = true;
		}
	}
	return new_angle;

}

void EKF::imu_callback(const sensor_msgs::ImuConstPtr& msg)
{
	IMUMeasurement z;

	z.t = msg->header.stamp;

	//transform the measurements into the body frame
	tf::Vector3 accel, omega,  accel_world;

	accel_world = tf::Vector3(this->state.ax(), this->state.ay(), this->state.az()); // the expected felt acceleration without gravity

	//form the world to base transform using the current angle
	tf::Matrix3x3 mat3;
	mat3.setRPY(this->state.thetax(), this->state.thetay(), this->state.thetaz());
	tf::Transform b2w = tf::Transform(mat3, tf::Vector3(this->state.x(), this->state.y(), this->state.z())).inverse();


	accel = tf::Vector3(msg->linear_acceleration.x - AX_BIAS, msg->linear_acceleration.y - AY_BIAS, msg->linear_acceleration.z - AZ_BIAS);
	omega = tf::Vector3(msg->angular_velocity.x - WX_BIAS, msg->angular_velocity.y - WY_BIAS, msg->angular_velocity.z - WZ_BIAS);


	tf::Vector3 accel_b, omega_b;

	accel_b = (this->base2imu * accel - this->base2imu * tf::Vector3(0, 0, 0));

	ROS_DEBUG_STREAM("imu base acceleration felt " << accel_b.x() << ", " << accel_b.y() << ", " << accel_b.z());

	accel_b = accel_b - (b2w * accel_world - b2w * tf::Vector3(0, 0, 0));

	ROS_DEBUG_STREAM("base acceleration felt without actual accel " << accel_b.x() << ", " << accel_b.y() << ", " << accel_b.z());

	omega_b = this->base2imu * omega - this->base2imu * tf::Vector3(0, 0, 0);

	double roll = atan2(-accel_b.x(), accel_b.z());
	double pitch = atan2(accel_b.y(), sqrt(accel_b.x()*accel_b.x() + accel_b.z()*accel_b.z()));

	ROS_DEBUG_STREAM("roll pitch: " << roll << ", " << pitch);

	z.z << roll, pitch, omega_b.x(), omega_b.y(), omega_b.z();

	double actual_accel_mag = accel_world.length();

	Eigen::Matrix<double, 5, 5> sig = Eigen::MatrixXd::Zero(5, 5);
	sig(0, 0) = msg->linear_acceleration_covariance.at(0) + actual_accel_mag * IMU_ANGLE_SIGMA_ACCEL_MULTIPLIER;
	sig(1, 1) = msg->linear_acceleration_covariance.at(4) + actual_accel_mag * IMU_ANGLE_SIGMA_ACCEL_MULTIPLIER;
	sig(2, 2) = msg->angular_velocity_covariance.at(0);
	sig(3, 3) = msg->angular_velocity_covariance.at(4);
	sig(4, 4) = msg->angular_velocity_covariance.at(8);

	z.Sigma = sig;

	// average imu_msgs if there is another
	for(auto e : this->measurements)
	{
		if(e.getType() == Measurement::IMU)
		{
			ROS_DEBUG_STREAM("averaging imu reading. z before: " << z.z);
			z.z = 0.5 * (z.z + e.getZ());
			ROS_DEBUG_STREAM("z after: " << z.z);
		}
	}

	//set h
	z.H = this->computeIMUMeasurementH();

	this->addMeasurement(z);

}

void EKF::mantis_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
	PoseMeasurement z;

	z.t = msg->header.stamp;

	double r, p, y;

	tf::Quaternion quat;
	quat.setValue(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
	tf::Matrix3x3 mat3 = tf::Matrix3x3(quat);
	mat3.getRPY(r, p, y);

	r = this->constrainAngle(r);
	p = this->constrainAngle(p);
	y = this->constrainAngle(y);

	z.z << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, r, p, y;

	z.Sigma = Eigen::MatrixXd::Zero(6, 6);

	z.Sigma(0, 0) = msg->pose.covariance.at(0*6 + 0);//xx
	z.Sigma(1, 1) = msg->pose.covariance.at(1*6 + 1);//yy
	z.Sigma(2, 2) = msg->pose.covariance.at(2*6 + 2);//zz

	z.Sigma(3, 3) = msg->pose.covariance.at(3*6 + 3);//thetax
	z.Sigma(4, 4) = msg->pose.covariance.at(4*6 + 4);//thetay
	z.Sigma(5, 5) = msg->pose.covariance.at(5*6 + 5);//thetaz

	//z.Sigma(3, 3) = (z.Sigma(6, 6)+z.Sigma(5, 5)+z.Sigma(4, 4)) / 3.0;//q0q0

	z.H = this->computePoseMeasurementH();

	this->addMeasurement(z);
}

void EKF::dipa_callback(const nav_msgs::OdometryConstPtr& msg)
{
	TwistMeasurement z;

	z.t = msg->header.stamp;

	z.z << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z, msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z;

	z.Sigma = Eigen::MatrixXd::Zero(6, 6);

	z.Sigma(0, 0) = msg->twist.covariance.at(0*6 + 0);//xx
	z.Sigma(1, 1) = msg->twist.covariance.at(1*6 + 1);//yy
	z.Sigma(2, 2) = msg->twist.covariance.at(2*6 + 2);//zz

	z.Sigma(3, 3) = msg->twist.covariance.at(3*6 + 3);//wxwx
	z.Sigma(4, 4) = msg->twist.covariance.at(4*6 + 4);//wywy
	z.Sigma(5, 5) = msg->twist.covariance.at(5*6 + 5);//wzwz


	this->addMeasurement(z);
}

State EKF::process(State prior, ros::Time t)
{
	double dt = (t - prior.t).toSec();

	State posterior;

	posterior.t = t;

	posterior.setPos((prior.getPos() + dt*prior.getVel() + 0.5*dt*dt*prior.getAccel()));

	posterior.setVel(prior.getVel() + dt*prior.getAccel());

	posterior.setAccel(prior.getAccel());

	posterior.setTheta((prior.getTheta() + prior.getOmega() * dt));

	//TODO constrain the theta

	posterior.setOmega(prior.getOmega());

	Eigen::Matrix<double, STATE_VECTOR_SIZE, STATE_VECTOR_SIZE> F = computeStateTransitionF(dt);

	posterior.Sigma = F * prior.Sigma * F.transpose() + computeStateProcessError(dt);

	return posterior;

}

State EKF::update(State prior, MeasurementCombination mats)
{
	Eigen::VectorXd y = mats.z - mats.H * prior.vec; //residual

	Eigen::MatrixXd S = mats.H * prior.Sigma * mats.H.transpose() + mats.Sigma;

	Eigen::MatrixXd K = prior.Sigma * mats.H.transpose() * S.lu().inverse();

	State posterior = prior;

	posterior.vec = prior.vec + K * y;
	posterior.Sigma = (Eigen::MatrixXd::Identity(STATE_VECTOR_SIZE, STATE_VECTOR_SIZE) - K * mats.H) * prior.Sigma;

	old_states.push_back(prior);

	return posterior;
}


State EKF::findClosestState(ros::Time t)
{
	double refT = t.toSec();
	bool decreasing = true;
	double lowestDelta = fabs(this->state.t.toSec() - refT);
	State bestState = this->state;

	for(std::vector<State>::iterator it = old_states.end() - 1; it != old_states.begin()-1; it--)
	{
		double thisDelta = fabs(it->t.toSec() - refT);

		if(thisDelta < lowestDelta)
		{
			lowestDelta = thisDelta;
			bestState = (*it);
		}
		else
		{
			ROS_DEBUG_STREAM("lowest delta_t abs: " << lowestDelta << " best state t: " << bestState.t.toSec() << " reference t: " << refT);
			break;
		}
	}

	return bestState;
}

Measurement EKF::predictMeasurementForward(Measurement z, ros::Time new_t){

	double dt = new_t.toSec() - z.getTime().toSec();

	//ROS_ASSERT(dt >= 0);

	if(dt == 0) // don't predict it if it is already new
	{
		return z;
	}

	if(dt < 0)
	{
		ROS_WARN_STREAM("Measurement is from the future with dt: " << dt);
	}

	if(z.getType() == Measurement::IMU)
	{
		// same measurement more variance
		Eigen::Matrix<double, 5, 5> R;
		R(0, 0) = dt*0.01;
		R(1, 1) = dt*0.01;
		R(2, 2) = dt*0.005;
		R(3, 3) = dt*0.005;
		R(4, 4) = dt*0.005;

		IMUMeasurement imuz;

		imuz.H = z.getH();
		imuz.z = z.getZ();
		imuz.t = new_t;
		imuz.Sigma = z.getSigma() + R;

		return Measurement(imuz);
	}
	else if(z.getType() == Measurement::POSE)
	{
		State reference = this->findClosestState(z.getTime());

		reference.setPos(Eigen::Vector3d(z.getZ()(0), z.getZ()(1), z.getZ()(2)));

		reference.setTheta(Eigen::Vector3d(z.getZ()(3), z.getZ()(4), z.getZ()(5)));

		reference = this->process(reference, reference.t + ros::Duration(dt));

		PoseMeasurement posez;

		posez.H = z.getH();
		posez.t = new_t;

		posez.z(0) = reference.x();
		posez.z(1) = reference.y();
		posez.z(2) = reference.z();
		posez.z(3) = reference.thetax();
		posez.z(4) = reference.thetay();
		posez.z(5) = reference.thetaz();

		Eigen::Matrix<double, 6, 6> R;
		R(0, 0) = dt*POSE_PREDICT_SIGMA;
		R(1, 1) = dt*POSE_PREDICT_SIGMA;
		R(2, 2) = dt*POSE_PREDICT_SIGMA;
		R(3, 3) = dt*POSE_PREDICT_SIGMA;
		R(4, 4) = dt*POSE_PREDICT_SIGMA;

		posez.Sigma = z.getSigma() + R;

		return Measurement(posez);
	}

	else if(z.getType() == Measurement::TWIST)
	{
		State reference = this->findClosestState(z.getTime());

		reference.setVel(Eigen::Vector3d(z.getZ()(0), z.getZ()(1), z.getZ()(2)));

		reference.setOmega(Eigen::Vector3d(z.getZ()(3), z.getZ()(4), z.getZ()(5)));

		reference = this->process(reference, reference.t + ros::Duration(dt));

		TwistMeasurement twistz;

		twistz.H = z.getH();
		twistz.t = new_t;

		twistz.z(0) = reference.dx();
		twistz.z(1) = reference.dy();
		twistz.z(2) = reference.dz();
		twistz.z(3) = reference.wx();
		twistz.z(4) = reference.wy();
		twistz.z(5) = reference.wz();

		Eigen::Matrix<double, 6, 6> R;
		R(0, 0) = dt*TWIST_PREDICT_SIGMA;
		R(1, 1) = dt*TWIST_PREDICT_SIGMA;
		R(2, 2) = dt*TWIST_PREDICT_SIGMA;
		R(3, 3) = dt*TWIST_PREDICT_SIGMA;
		R(4, 4) = dt*TWIST_PREDICT_SIGMA;
		R(5, 5) = dt*TWIST_PREDICT_SIGMA;

		twistz.Sigma = z.getSigma() + R;

		return Measurement(twistz);
	}
	else
	{
		ROS_FATAL("invalid type for measurement interpolation!");
		return Measurement();
	}
}

void EKF::predictAllMeasurementsForward(ros::Time new_t)
{
	for(auto& e : this->measurements)
	{
		e = this->predictMeasurementForward(e, new_t);
	}
}

void EKF::addMeasurement(Measurement z){
#if REMOVE_SIMILAR_MEASUREMENTS
	bool pass = false;
	while(!pass)
	{
		pass = true;
		for(std::deque<Measurement>::iterator it = this->measurements.begin(); it != this->measurements.end(); it++)
		{
			if(it->getType() == z.getType())
			{
				pass = false;
				ROS_DEBUG_STREAM("erasing measurement size before: " << this->measurements.size());
				this->measurements.erase(it);
				ROS_DEBUG_STREAM("after size: " << this->measurements.size());
			}
		}
	}
#endif

	this->measurements.push_back(z);

	ROS_DEBUG_STREAM("ADDED MEASUREMENT TYPE: " << z.getType() <<"\nz: " << z.getZ().transpose() <<"\nsigma: " << z.getSigma() << "\nt: " << z.getTime() << "\nH: " << z.getH());

}

