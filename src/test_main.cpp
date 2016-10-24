
// Runs the top level loop calling the Filter Class and then initating particle objects and iterating through them.

#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include "particle_filter.h"

using namespace std;

//#define TEST_IMPORTANCE_SAMPLING 
#define SENSOR_MODEL

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

		// ParticleFilter(int num_particles, double motion_mean, double motion_sigma, double laser_mean, double laser_sigma) {

		// 	num_particles_ = num_particles;
		// 	motion_mean_ = motion_mean;
		// 	motion_sigma_ = motion_sigma;
		// 	laser_mean_ = laser_mean;
		// 	laser_sigma_ = laser_sigma;
		// }



	#ifdef SENSOR_MODEL
		std::vector<Pose> traj;
		int num_particles=1000;
		cout<<"Hey yaa testing Sensor Model! No. of particles are = "<< num_particles << endl;
		ParticleFilter filter_obj {num_particles,0,.1,0,.1};
		//ParticleFilter pf(1e4, 0, 0.1, 0, 0.1);
		filter_obj.ReadData("../data/robotdata1.log", "../data/wean.dat");
		filter_obj.Filter(traj);

	#endif







 	return (0);



	//test code
	// Eigen::MatrixXd m(10,10);
	// Eigen::MatrixXd m1= Eigen::MatrixXd::Constant(3,3,1.0);
	// cout<< m.row(1)[1]<<endl;
	// int a[20];
	// std::fill_n(a,20,-3);
	// cout << a[2] <<endl;


}
