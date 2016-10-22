
// Runs the top level loop calling the Filter Class and then initating particle objects and iterating through them.

#include <iostream>
#include <vector>
#include <string>
#include "particle_filter.h"

using namespace std;

int main (int argc , char ** argv )

{

	ParticleFilter pf(1e4, 0, 0.1, 0, 0.1);
	pf.ReadData("../data/robotdata1.log", "../data/wean.dat");

	return (0);
	
}