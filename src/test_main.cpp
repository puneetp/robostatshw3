
// Runs the top level loop calling the Filter Class and then initating particle objects and iterating through them.

#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include "particle_filter.h"

using namespace std;

#define TEST_IMPORTANCE_SAMPLING 

int main (int argc , char ** argv )

{

	// ParticleFilter pf(1e4, 0, 0.1, 0, 0.1);
	// pf.ReadData("../data/robotdata1.log", "../data/wean.dat");

	// Test Importance Sampling
	#ifdef TEST_IMPORTANCE_SAMPLING
		const unsigned int num_particles = 5; // also change weights
		ParticleFilter pf(num_particles, 0, 0.1, 0, 0.1);
		
		// Print All Particles (use theta as ID)
		cout << "Before = " << endl;
		for (int i=0; i<num_particles; i++){
			cout << pf.particles_[i].GetPose().theta << endl;
		}

		// Run Importane Sampling
		std::vector<double> weights {10, 5, 1, 1, 1};
	    for (int i=0; i<num_particles; i++){
	    	pf.particles_[i].SetWeight(weights[i]);	
	    }
		pf.ImportanceSampling(pf.particles_);

	    // Print All Particles
		cout << "After = " << endl;
		for (int i=0; i<num_particles; i++){
			cout << pf.particles_[i].GetPose().theta << endl;
		}
	    	
	#endif

	// Test Something Else

 	return (0);



	//test code
	// Eigen::MatrixXd m(10,10);
	// Eigen::MatrixXd m1= Eigen::MatrixXd::Constant(3,3,1.0);
	// cout<< m.row(1)[1]<<endl;
	// int a[20];
	// std::fill_n(a,20,-3);
	// cout << a[2] <<endl;


}
