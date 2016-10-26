
// Runs the top level loop calling the Filter Class and then initating particle objects and iterating through them.

#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include "particle_filter.h"

using namespace std;

int main (int argc , char ** argv )
{
	 std::vector<Pose> traj;
 	// ParticleFilter pf(1e2, 0, 0.1, 0, 15);
	 ParticleFilter pf(1e4, 0, 1e-4, 0, 15, "../data/robotdata1.log", "../data/wean.dat");
	 // ParticleFilter pf(10, 0, 1e-4, 0, 15, "../data/robotdata1.log", "../data/wean.dat");
	 // pf.ReadData("../data/robotdata1.log", "../data/wean.dat");
	 pf.Filter(traj);
	 // pf.TestMotionModel();

	 return (0);
}