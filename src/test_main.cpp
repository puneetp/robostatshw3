
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

	// std::vector<Pose> traj;
 // 	ParticleFilter pf(1e4, 0, 0.1, 0, 0.1);
 // 	pf.ReadData("../data/robotdata1.log", "../data/wean.dat");
 // 	pf.Filter(traj);

	// Test Importance Sampling
	#ifdef TEST_IMPORTANCE_SAMPLING
		std::vector<Pose> traj_tsi;
		const unsigned int num_particles = 5; // also change weights
		ParticleFilter pf_tsi(num_particles, 0, 0.1, 0, 0.1);
	 	pf_tsi.ReadData("../data/robotdata1.log", "../data/wean.dat");
 		pf_tsi.Filter(traj_tsi);
		
		std::cout << std::endl;
		std::cout << "<===== Importance Sampling Output =====>" << std::endl;
		// Print All Particles (use theta as ID)
		std::cout << "Before = ";
		for (int i=0; i<num_particles; i++){
			cout << pf_tsi.particles_[i].GetPose().theta << ",";
		}
		std::cout << std::endl;
		// Run Importane Sampling
		std::vector<double> weights {0.1, 0.2, 1, 0.1, 0.2};
		cout << "Old Weights = ";
	    for (int i=0; i<num_particles; i++){
	    	pf_tsi.particles_[i].SetWeight(weights[i]);	
	    	cout << pf_tsi.particles_[i].GetWeight() << ",";	
	    }

	    std::cout << std::endl;
		pf_tsi.ImportanceSampling(pf_tsi.particles_, 1);
		
	    // Print All Particles and New weights
		cout << "After = ";
		for (int i=0; i<num_particles; i++){
			cout << pf_tsi.particles_[i].GetPose().theta << ",";
		}
		std::cout << std::endl;
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