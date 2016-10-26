
// Runs the top level loop calling the Filter Class and then initating particle objects and iterating through them.

#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include "particle_filter.h"

using namespace std;


// #define TEST_IMPORTANCE_SAMPLING 
// #define SENSOR_MODEL
#define TEST_MAP_DISPLAY


int main (int argc , char ** argv )
{

	std::vector<Pose> traj;
 	// ParticleFilter pf(1e2, 0, 0.1, 0, 15);
	ParticleFilter pf(1e4, 0, 1e-4, 0, 15, "../data/robotdata1.log", "../data/wean.dat");
 	// pf.ReadData("../data/robotdata1.log", "../data/wean.dat");
 	pf.Filter(traj);

	// Test Importance Sampling
	#ifdef TEST_IMPORTANCE_SAMPLING
		std::cout << std::endl;
		std::cout << "<===== Importance Sampling Output =====>" << std::endl;
		
		std::vector<Pose> traj_tsi;
		const unsigned int num_particles = 5; // also change weights
		ParticleFilter pf_tsi(num_particles, 0, 0.1, 0, 0.1);
	 	pf_tsi.ReadData("../data/robotdata1.log", "../data/wean.dat");
 		pf_tsi.Filter(traj_tsi);

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

	#ifdef TEST_MAP_DISPLAY
		std::cout << std::endl;
		std::cout << "<===== Test Map Display Output =====>" << std::endl;
		for (int i=0; i<100; i++){
			pf.UpdateDisplay();
		}
		// char crap;
		// std::cin >> crap;
		std::cout << std::endl;		
	#endif


		// ParticleFilter(int num_particles, double motion_mean, double motion_sigma, double laser_mean, double laser_sigma) {

		// 	num_particles_ = num_particles;
		// 	motion_mean_ = motion_mean;
		// 	motion_sigma_ = motion_sigma;
		// 	laser_mean_ = laser_mean;
		// 	laser_sigma_ = laser_sigma;
		// }
	
	#ifdef SENSOR_MODEL
		std::vector<Pose> traj2;
		int num_particles=1000;
		cout<<"Hey yaa testing Sensor Model! No. of particles are = "<< num_particles << endl;
		ParticleFilter filter_obj {num_particles,0,.1,0,.1};
		//ParticleFilter pf(1e4, 0, 0.1, 0, 0.1);
		filter_obj.ReadData("../data/robotdata1.log", "../data/wean.dat");
		filter_obj.Filter(traj2);

	#endif

 	return (0);

	// Test Something Else
	// #ifdef SOMETHING
	// 	std::cout << std::endl;
	// 	std::cout << "<===== Something Else Output =====>" << std::endl;

	// 	std::cout << std::endl;
	// #endif


	//test code
	// Eigen::MatrixXd m(10,10);
	// Eigen::MatrixXd m1= Eigen::MatrixXd::Constant(3,3,1.0);
	// cout<< m.row(1)[1]<<endl;
	// int a[20];
	// std::fill_n(a,20,-3);
	// cout << a[2] <<endl;
}
