#include <random>
#include "particle_filter.h"

Eigen::MatrixXd ParticleFilter::
ComputeTransform(double x, double y, double theta) {
	Eigen::MatrixXd T(3, 3);
	T << 	
		std::cos(theta), -std::sin(theta), x,
		std::sin(theta), std::cos(theta), y,
		0, 0, 1;

	return T;
}

Eigen::MatrixXd ParticleFilter::
NoisyTransform(Eigen::MatrixXd T1, Eigen::MatrixXd T2) {
	// Get displacements
	Eigen::MatrixXd dT = T1.inverse() * T2;
	double dx = dT(0, 2);
	double dy = dT(1, 2);
	double dtheta = std::atan2(-dT(0, 1), dT(0, 0));

	// Add some noise
	std::default_random_engine generator;
	std::normal_distribution<double> dist_x(dx, motion_sigma_);
	std::normal_distribution<double> dist_y(dy, motion_sigma_);
	std::normal_distribution<double> dist_theta(dtheta, motion_sigma_);

	// build noisy transform matrix
	dx = dist_x(generator);
	dy = dist_y(generator);
	dtheta = dist_theta(generator);

	return ComputeTransform(dx, dy, dtheta);
}

void ParticleFilter::
MotionModel(Particle &p, Pose &pos0, Pose &pos1) {
	Eigen::Vector3d new_pose, old_pose;
	Pose particle_pose = p.GetPose();
	old_pose << particle_pose.x, particle_pose.y, particle_pose.theta;

	new_pose = NoisyTransform(
			ComputeTransform(pos0.x, pos0.y, pos0.theta),
			ComputeTransform(pos1.x, pos1.y, pos1.theta)
		) * old_pose;

	p.SetPose(new_pose(0), new_pose(1), new_pose(2));
}