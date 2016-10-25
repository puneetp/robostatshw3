
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
	 ParticleFilter pf(5*1e3, 0, 1e-4, 0, 30);
	 pf.ReadData("../data/robotdata1.log", "../data/wean.dat");
	 pf.Filter(traj);
	 // pf.TestMotionModel();

	 return (0);
}