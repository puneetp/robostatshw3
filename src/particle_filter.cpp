#include <random>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include "particle_filter.h"
#include "math.h"

#define NUM_ODOM 			4
#define NUM_LASER			187

Eigen::Matrix3d ParticleFilter::
ComputeTransform(double x, double y, double theta) {
	Eigen::Matrix3d T;
	T <<
		std::cos(theta), -std::sin(theta), x,
		std::sin(theta), std::cos(theta), y,
		0, 0, 1;

	return T;
}

Eigen::Matrix3d ParticleFilter::
NoisyTransform(Eigen::Matrix3d T1, Eigen::Matrix3d T2) {
	// Get displacements
	Eigen::Matrix3d dT = T1.inverse() * T2;
	double dx = dT(0, 2);
	double dy = dT(1, 2);
	double dtheta = std::atan2(-dT(0, 1), dT(0, 0));

	// Add some noise
	std::default_random_engine generator;
	std::normal_distribution<double> dist_x(dx, motion_sigma_);
	std::normal_distribution<double> dist_y(dy, motion_sigma_);
	std::normal_distribution<double> dist_theta(dtheta, motion_sigma_);

	// build noisy transform matrix
	dx = dist_x(generator);
	dy = dist_y(generator);
	dtheta = dist_theta(generator);

	return ComputeTransform(dx, dy, dtheta);
}

void ParticleFilter::
MotionModel(Particle &p, Eigen::Matrix3d T1, Eigen::Matrix3d T2) {
	Eigen::Matrix3d new_T, old_T;
	old_T << p.GetT();

	new_T = NoisyTransform(T1, T2) * old_T;

	p.SetT(new_T);
	p.SetPose(new_T(0, 2), new_T(1, 2), std::atan2(new_T(1, 0), new_T(0, 0)));
}

void ParticleFilter::
ReadData(char* data_file, char *map_file) {
	// read laser and odom data
	laser_data_.rows = 0;
	odom_data_.rows = 0;
	laser_data_.data.resize(4000, NUM_LASER);
	odom_data_.data.resize(4000, NUM_ODOM);
	FILE *f = fopen(data_file, "r");
	char str[1024], ch;
	double temp = 5.;
	int offset = 0, readCharCount;

	while(fgets(str, 1024, f) != NULL) {
		offset = 1;
		sscanf(str, "%c ", &ch);
		if(ch == 'O') {
			// odom log
			for(size_t i = 0; i < NUM_ODOM-1; ++i) {
				sscanf(str+offset, "%lf %n", &temp, &readCharCount);
				offset += readCharCount;
				odom_data_.data(odom_data_.rows, i) = temp;
			}
			// last line
			sscanf(str+offset, "%lf\n%n", &temp, &readCharCount);
			offset += readCharCount;
			odom_data_.data(odom_data_.rows, NUM_ODOM-1) = temp;
			++odom_data_.rows;
		}
		else if(ch == 'L') {
			// laser log
			for(size_t i = 0; i < NUM_LASER-1; ++i) {
				sscanf(str+offset, "%lf %n", &temp, &readCharCount);
				offset += readCharCount;
				laser_data_.data(laser_data_.rows, i) = temp;
			}
			// last line
			sscanf(str+offset, "%lf\n%n", &temp, &readCharCount);
			offset += readCharCount;
			laser_data_.data(laser_data_.rows, NUM_LASER-1) = temp;
			++laser_data_.rows;
		}
	}

	// Read map
	read_beesoft_map(map_file, &map_);

	std::cout << "Num odom entries: " << odom_data_.rows << "\n";
	std::cout << "Num laser entries: " << laser_data_.rows << "\n";
}

void ParticleFilter::
Filter(std::vector<Pose> &trajectory) {
	int laser_idx(1), odom_idx(0);
	Eigen::Matrix3d prev_T, curr_T;

	// Initialize with first odom entry
	prev_T = ComputeTransform(odom_data_.data(0, 0), odom_data_.data(0, 1), odom_data_.data(0, 2));
	++odom_idx;

	while(1) {
		// Read log file. Let's stick with laser for now?
		curr_T = ComputeTransform(laser_data_.data(laser_idx, 0), laser_data_.data(laser_idx, 1), laser_data_.data(laser_idx, 2));

		// Apply motion and sensor model on all particles
		for(int i = 0; i < num_particles_; ++i) {
			// motion model
			MotionModel(particles_[i], prev_T, curr_T);

			// sensor model
			// SensorModel(particles_[i], laser_idx);
		}

		// Importance sampling
		laser_idx++;
	}
}

/* Regenerates the particles_ according to weights
 */
void ParticleFilter::ImportanceSampling(std::vector<Particle> &particles){
	
	// Compute vector of cumulative weights
	const unsigned int num_particles = particles.size();
	std::vector<double> cum_weights(num_particles+1, 0.0);
	for (int i=1; i<num_particles; i++){
		cum_weights[i] = cum_weights[i-1] + particles[i].GetWeight();
	}

	// Generate new particles
	std::vector<Particle> new_particles;
	std::default_random_engine generator(time(0));
	std::uniform_real_distribution<float> distribution(0.0, cum_weights[num_particles]);
	for (int i=0; i<num_particles; i++){
		double random_no = distribution(generator);
		auto lower = std::lower_bound(cum_weights.begin(), cum_weights.end(), random_no);
		unsigned int index = lower - cum_weights.begin();
		new_particles.push_back(particles[index]);
	}

	// Set weights to zero
	
	// Replace particles_ with new particles
	particles = new_particles;
}

double ParticleFilter::SensorModel( Particle & p , int laser_index)

{
	//laser_data_.row(laser_index)[]


   //Check if you are in greay space , if yes return 0 weight
   //For the values X and Y and Tetha project Laser out into the MAP.
   //Read the MAP in these laser rays and get back values of first obstruction as per the map,
   //...rest weight out the gray values if they come before.
   //Use the value to a) Map Reading of lasters b) Laser reading to fetch value c) The value fetched call this in loop 180 times
   //Return the sum of log of all the probabilties ( CHECK again here for what calculation has to be done.)

	int map_directed_obstacle_range[180];
	double per_particle_sensor_probability_vector[180];

	//Initializing
	int search_increment=5;
	double threshold_for_obstacle=0.7;
	int hop=5;// How many lasers do we want to hop in search space
	std::fill_n(map_directed_obstacle_range,180,-1); // to -1


	//Running accross all laswers
	for ( int i ; i<180;i+hop)
	{
		int x=0;
		int y=0;
		int r=0;
		while (x<8000 & y <8000)
		{

			double offset_phi = i*M_PI/180.0; // here degree

			x=std::floor(r*std::cos(p.GetPose().theta+offset_phi));

			y=std::floor(r*std::sin(p.GetPose().theta+offset_phi));

			double obstacle_prob=map_.prob[x][y];

			if (obstacle_prob > threshold_for_obstacle) // Note we check for certain threshold
			{
				map_directed_obstacle_range[i]=r;
				break; // check this command
			}
			r=r+search_increment;
		}
	}


	// call the function to generate sensor probability per laser
    ProbabilityDistributionFunction( map_directed_obstacle_range, hop, laser_index, per_particle_sensor_probability_vector );

    double weight_log=0.0;

	for (int j =0 ; j < 180 ; j++)
	{
		weight_log=std::log(per_particle_sensor_probability_vector[j])+weight_log;
	}

	double weight_value= std::exp(weight_log);  //actual weight

	return weight_value;

}


// Matlab equivalent is one line plot /(x,max(exp(-x),normpdf(x,9,.5))) // produce medium peaky graphs

void ParticleFilter::ProbabilityDistributionFunction( int map_directed_obstacle_range[],int hop,int laser_index,double per_particle_sensor_probability_vector[] )
{

	double laser_mean;
	double laser_std=0.5; //editable
	double min_probability_setting=0.02; //editable
	double exp_value=0.0;
	double normal_value=0.0;

	for( int i ; i < 180 ; i++ )
	{
		if (map_directed_obstacle_range[i]!=-1)
		{
			//create normal distribution
			laser_mean=map_directed_obstacle_range[i];

			// fetch reading at value
			double fetch_laser_value_at_this_point = laser_data_.data.row(laser_index)[i]; // REVIEW this

			//create normal distribution
			std::normal_distribution<double> distribution (laser_mean, laser_std);

			// From Normal Distrubution of PDF
			// = distribution(fetch_laser_value_at_this_point);
			normal_value= (1/(laser_std*std::sqrt(2*M_PI)))*std::exp(-.5*std::pow( ((fetch_laser_value_at_this_point - laser_mean)/laser_std)  ,2))      ;

			// From Exponential Function of PDF , since its falling , it e^(-x)
			exp_value=std::exp(-1.0*fetch_laser_value_at_this_point);

			// add some minor noise
			double max_step1=std::max(exp_value,min_probability_setting);
			// get the maximum of the two
			per_particle_sensor_probability_vector[i]=std::max(max_step1,normal_value);
		}
	}

}








