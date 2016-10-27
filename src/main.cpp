
// Runs the top level loop calling the Filter Class and then initating particle objects and iterating through them.

#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include "particle_filter.h"
#include <limits>

using namespace std;

int main (int argc , char ** argv )
{

	 std::vector<Pose> traj;
 	// ParticleFilter pf(1e2, 0, 0.1, 0, 15);
	//ParticleFilter pf(5*1e2, 0, .01 , 0, 50); // puneet
	 // ParticleFilter pf(5*1e2, 0, 1e-4, 0, 30, "../data/robotdata1.log", "../data/wean.dat");

	 ParticleFilter pf(80000, 0, 6, 0, 200, "../data/robotdata1.log", "../data/wean.dat");

	// ParticleFilter pf(1e4, 0, 1e-2, 0, 200, "../data/robotdata1.log", "../data/wean.dat");

	 // pf.ReadData("../data/robotdata1.log", "../data/wean.dat");

	 pf.Filter(traj);
	 // pf.Filter_new(traj);
	 // pf.Filter(traj);
	 // pf.TestRotMotionModel();
	 // pf.TestMotionModel();
	 return (0);
}


	// Puneet working params 
	// ParticleFilter pf(1000, 0, 6, 0, 200, "../data/robotdata1.log", "../data/wean.dat");
