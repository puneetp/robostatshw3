
// Runs the top level loop calling the Filter Class and then initating particle objects and iterating through them.

#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include "particle_filter.h"

using namespace std;

int main (int argc , char ** argv )

{

	

	 ParticleFilter pf(1e4, 0, 0.1, 0, 0.1);
	 pf.ReadData("../data/robotdata1.log", "../data/wean.dat");

	 return (0);



	//test code
	// Eigen::MatrixXd m(10,10);
	// Eigen::MatrixXd m1= Eigen::MatrixXd::Constant(3,3,1.0);
	// cout<< m.row(1)[1]<<endl;
	// int a[20];	
	// std::fill_n(a,20,-3);
	// cout << a[2] <<endl;

	
}