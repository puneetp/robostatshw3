#include "particle.h"
#include "data_structs.h"

void Particle::
SetPose(double x, double y, double theta) {
	pose_.x = x;
	pose_.y = y;
	pose_.theta = theta;
}

void Particle::
SetWeight(double w) {
	w_ = w;
}

Pose Particle::
GetPose() {
	return pose_;
}

double Particle::
GetWeight() {
	return w_;
}