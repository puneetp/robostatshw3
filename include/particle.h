#ifndef PARTICLE_H
#define PARTICLE_H

#include "data_structs.h"

/** Defines the Particle class */

class Particle {
public:

	// Constructor
	Particle(void) {
		pose_.x = pose_.y = pose_.theta = 0.;
		w_ = 0.;
	}

	void SetPose(double x, double y, double theta);

	void SetWeight(double w);

	Pose GetPose();

	double GetWeight();

	Eigen::Matrix3d GetT();

	void SetT(Eigen::Matrix3d T);

private:
	/* ****************** Member variables ********************************* */
	Pose pose_;
	double w_;
	Eigen::Matrix3d T_;
};

#endif