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

	if(!test)
	{
		ros::spin();
	}
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

State EKF::process(State prior, ros::Time t)
{
	double dt = (t - prior.t).toSec();

	State posterior;

	posterior.t = t;

	posterior.setPos(prior.getPos() + dt*prior.getVel() + 0.5*dt*dt*prior.getAccel());

	posterior.setVel(prior.getVel() + dt*prior.getAccel());

	posterior.setAccel(prior.getAccel());

	double theta = prior.getOmega().norm() * dt;

	Eigen::Vector3d omega_hat = prior.getOmega() / (theta/dt);

	Eigen::Quaterniond dq;

	if(theta == 0)
	{
		dq = Eigen::Quaterniond(cos(theta/2.0), 0, 0, 0);
	}
	else
	{
		double st2 = sin(theta/2.0);
		dq = Eigen::Quaterniond(cos(theta/2.0), omega_hat.x()*st2, omega_hat.y()*st2, omega_hat.z()*st2);
	}

	posterior.setQuat(prior.getQuaternion() * dq);

	posterior.setOmega(prior.getOmega());

	Eigen::Matrix<double, 16, 16> F = computeStateTransitionJacobian(prior, dt);

	posterior.Sigma = F * prior.Sigma * F.transpose() + computeStateProcessError(dt);

	return posterior;

}

State EKF::update(State prior, MeasurementCombination mats)
{
	//Eigen::Matrix<double, mats.z.rows(), 1> = mats.z - mats
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

}

void EKF::addMeasurement(Measurement z){
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
}

