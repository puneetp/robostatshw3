#include <random>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include "particle_filter.h"
#include "math.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>

#define NUM_ODOM 			4
#define NUM_LASER			187
#define LASER_POD_FORWARD_OFFSET   25 // This is in 
#define UNOCCUPIED_TOL		1e-2

/** Constructor */
ParticleFilter::
ParticleFilter(int num_particles, double motion_mean, double motion_sigma,
	double laser_mean, double laser_sigma) {
	num_particles_ = num_particles;
	motion_mean_ = motion_mean;
	motion_sigma_ = motion_sigma;
	laser_mean_ = laser_mean;
	laser_sigma_ = laser_sigma;
	cv::namedWindow("ParticleFilter", CV_WINDOW_NORMAL);
}

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
	// return T2 * T1.inverse();

	// Get displacements
	Eigen::Matrix3d dT = T2 * T1.inverse();
	double dx = dT(0, 2);
	double dy = dT(1, 2);
	double dtheta = std::atan2(-dT(0, 1), dT(0, 0));

	// Add some noise
	std::default_random_engine generator(time(0));
	std::normal_distribution<double> dist_x(dx, motion_sigma_);
	std::normal_distribution<double> dist_y(dy, motion_sigma_);
	std::normal_distribution<double> dist_theta(dtheta, motion_sigma_);

	// build noisy transform matrix65
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

//

void ParticleFilter::
Filter(std::vector<Pose> &trajectory) {
	int laser_idx(1), odom_idx(0);
	Eigen::Matrix3d prev_T, curr_T;

	// Preprocess map
	PreprocessMap();

	// Initialize particles
	InitParticles();

	DumpParticlesToFile();
	DumpOdomToFile();
	DumpLaserToFile();

	UpdateDisplay();

	// // Initialize with first odom entry
	// prev_T = ComputeTransform(odom_data_.data(0, 0), odom_data_.data(0, 1), odom_data_.data(0, 2));
	// ++odom_idx;


	// while(1) {
	// 	// Read log file. Let's stick with laser for now?
	// 	curr_T = ComputeTransform(laser_data_.data(laser_idx, 0), laser_data_.data(laser_idx, 1), laser_data_.data(laser_idx, 2));

	// 	// Apply motion and sensor model on all particles
	// 	for(int i = 0; i < num_particles_; ++i) {
	// 		// motion model
	// 		MotionModel(particles_[i], prev_T, curr_T);

	// 		// sensor model
	// 		// SensorModel(particles_[i], laser_idx);
	// 	}

	// 	// Importance sampling
		// ImportanceSampling(particles_)
	// 	laser_idx++;		
	//  pf.UpdateDisplay();
	// }

}

/* Regenerates the particles_ according to weights
 */
void ParticleFilter::ImportanceSampling(std::vector<Particle> &particles , int verbose){
	
	// Compute vector of cumulative weights
	const unsigned int num_particles = particles.size();
	std::vector<double> cum_weights(num_particles+1, 0.0);
	for (int i=1; i<num_particles+1; i++){
		cum_weights[i] = cum_weights[i-1] + fabs(particles[i-1].GetWeight());
	}

	// Generate new particles
	std::vector<Particle> new_particles;
	std::default_random_engine generator(time(0));
	std::uniform_real_distribution<float> distribution(0.0, cum_weights[num_particles]);
	
	if (verbose){
		std::cout << "cum_weights = ";
		for(auto it = cum_weights.begin(); it != cum_weights.end(); it++){
			std::cout<< *it << ",";
	    }
		std::cout << std::endl;
	}

	for (int i=0; i<num_particles; i++){
		double random_no = distribution(generator);
		auto lower = std::lower_bound(cum_weights.begin(), cum_weights.end(), random_no);
		unsigned int index = lower - cum_weights.begin() - 1; // -1 => caz weight has more 1 more element
		new_particles.push_back(particles[index]);
		if (verbose){
			std::cout << "Random no= " << random_no << " index= " << index << std::endl;
		}
	}
	std::cout << std::endl;

	// Replace particles_ with new particles
	particles = new_particles;
}

double ParticleFilter::SensorModel( Particle & p , int laser_row_index)

{
	//laser_data_.row(laser_row_index)[]


   //Check if you are in greay space , if yes return 0 weight
   //For the values X and Y and Tetha project Laser out into the MAP.
   //Read the MAP in these laser rays and get back values of first obstruction as per the map,
   //...rest weight out the gray values if they come before.
   //Use the value to a) Map Reading of lasters b) Laser reading to fetch value c) The value fetched call this in loop NUM_LASER times
   //Return the sum of log of all the probabilties ( CHECK again here for what calculation has to be done.)
	 
	int map_directed_obstacle_range[NUM_LASER];
	double per_particle_sensor_probability_vector[NUM_LASER];

	//Initializing
	int search_increment=5;
	double threshold_for_obstacle=0.7;

	int hop=5;// How many lasers do we want to hop in search space 
	std::fill_n(map_directed_obstacle_range,NUM_LASER,-1); // to -1 
	

	//Running accross all lasers 
	for ( int i ; i<NUM_LASER;i+hop)
	{
		int x=0;
		int y=0;
		int r=0;
		while ( ((x<8000)&&(y<8000))  &&  ((x>0)&&(y>0))  )  
		{

			double increment_scan_angle_phi = i*M_PI/180.0; // Angle of laser from start

			double start_offset_laser = -90.0*M_PI/180.0; // starting point -90 degree

			double robot_theta= p.GetPose().theta;		

			double rx=r*std::cos(robot_theta + increment_scan_angle_phi + start_offset_laser);

			double ry=r*std::sin(robot_theta + increment_scan_angle_phi+start_offset_laser);
			
			double laser_x_offset = LASER_POD_FORWARD_OFFSET*std::cos(robot_theta);

			double laser_y_offset = LASER_POD_FORWARD_OFFSET*std::sin(robot_theta);

			int true_x= std::floor(p.GetPose().x + rx + laser_x_offset);

			int true_y= std::floor(p.GetPose().y + ry + laser_y_offset);

			double obstacle_prob=map_.prob[true_x][true_y];						

			if (obstacle_prob > threshold_for_obstacle) // Note we check for certain threshold
			{
				map_directed_obstacle_range[i]=r;
				break; // check this command
			}

			r=r+search_increment;
		}
	}

	//Call the function to generate laser probabilities per particle.	
    Sensor_models_laser_PDF_vector( map_directed_obstacle_range, hop, laser_row_index, per_particle_sensor_probability_vector );

    double weight_log=0.0;

	for (int j =0 ; j < NUM_LASER ; j++)
	{
		weight_log=std::log(per_particle_sensor_probability_vector[j])+weight_log;
	}

	double weight_value= std::exp(weight_log);  //actual weight

	return weight_value;

}


// Matlab equivalent is one line plot /(x,max(exp(-x),normpdf(x,9,.5))) // produce medium peaky graphs

void ParticleFilter::Sensor_models_laser_PDF_vector( int map_directed_obstacle_range[],int hop,int laser_row_index,double per_particle_sensor_probability_vector[] )
{

	double laser_mean;
	double laser_std=0.5; //editable
	double min_probability_setting=0.02; //editable
	double exp_value=0.0;
	double normal_value=0.0;
	
	for( int i ; i < NUM_LASER ; i++ )

	{
		if (map_directed_obstacle_range[i]!=-1)
		{
			//create normal distribution
			laser_mean=map_directed_obstacle_range[i];

			// fetch reading at value
			double fetch_laser_value_at_this_point = laser_data_.data.row(laser_row_index)[i]; // REVIEW this

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

void ParticleFilter::
PreprocessMap() {
	// iterate through the map and find unoccupied cells
	for(int i = 0; i < map_.size_y; ++i) {
		for(int j = 0; j < map_.size_x; ++j) {
			if(map_.prob[i][j] > -0.1 && map_.prob[i][j] < UNOCCUPIED_TOL) {
				// add to list
				MapCell cell;
				cell.row = i; cell.col = j;
				unoccupied_list_.push_back(cell);
			}			
		}
	}
}

void ParticleFilter::
InitParticles() {
	std::default_random_engine generator;
	std::uniform_int_distribution<int> disc_distribution(0, unoccupied_list_.size()-1);
	std::uniform_real_distribution<double> real_distribution(0.0, 2*M_PI);

	int cell_idx;
	double theta, x, y;

	for(int i = 0; i < num_particles_; ++i) {
		Particle p;
		// randomly choose an unoccupied cell to place this particle in
		cell_idx = disc_distribution(generator);

		// randomly choose an orientation
		theta = real_distribution(generator);

		// Place the particle in the center of the cell (correct?)
		x = unoccupied_list_[cell_idx].col * map_.resolution + map_.resolution/2;
		y = unoccupied_list_[cell_idx].row * map_.resolution + map_.resolution/2;

		p.SetPose(x, y, theta);
		p.SetT(ComputeTransform(x, y, theta));

		particles_.push_back(p);
	}
}


/******************** test code *****************************************/
void ParticleFilter::
DumpParticlesToFile() {
	FILE *f = fopen("particles.csv", "w");

	for(int i = 0; i < num_particles_; ++i) {
		Pose p = particles_[i].GetPose();
		fprintf(f, "%lf,%lf,%lf\n", p.x, p.y, p.theta);
	}

	fclose(f);
}

void ParticleFilter::
DumpOdomToFile() {
	FILE *f = fopen("odom.csv", "w");

	for(int i = 0; i < odom_data_.rows; ++i) {
		fprintf(f, "%lf,%lf,%lf,%lf\n", odom_data_.data(i, 0), odom_data_.data(i, 1),
			odom_data_.data(i, 2), odom_data_.data(i, 3));
	}

	fclose(f);
}

void ParticleFilter::
DumpLaserToFile() {
	FILE *f = fopen("laser.csv", "w");

	for(int i = 0; i < laser_data_.rows; ++i) {
		for(int j = 0; j < NUM_LASER-1; ++j) {
			fprintf(f, "%lf,", laser_data_.data(i, j));
		}
		fprintf(f, "%lf\n", laser_data_.data(i, NUM_LASER-1));
	}

	fclose(f);
}

void ParticleFilter::
TestMotionModel() {
	FILE *f = fopen("particle_traj.csv", "w");
	Pose pose;
	int odom_idx = 0;
	Eigen::Matrix3d curr_T, prev_T;

	// Initialize particle to the first odom reading
	Particle p;
	p.SetPose(odom_data_.data(0, 0), odom_data_.data(0, 1), odom_data_.data(0, 2));
	p.SetT(ComputeTransform(odom_data_.data(0, 0), odom_data_.data(0, 1), odom_data_.data(0, 2)));
	num_particles_ = 1;
	std::vector<Particle> temp;
	temp.push_back(p);
	particles_ = temp;

	prev_T = ComputeTransform(odom_data_.data(0, 0), odom_data_.data(0, 1), odom_data_.data(0, 2));

	++odom_idx;

	while(odom_idx <= odom_data_.rows-1) {
		curr_T = ComputeTransform(odom_data_.data(odom_idx, 0), odom_data_.data(odom_idx, 1), odom_data_.data(odom_idx, 2));
		MotionModel(p, prev_T, curr_T);
		pose = p.GetPose();
		fprintf(f, "%lf,%lf,%lf\n", pose.x, pose.y, pose.theta);
		prev_T << curr_T;
		++odom_idx;
		UpdateDisplay();
	}

	fclose(f);
}

/** Updates the visualization of the map */
void ParticleFilter::UpdateDisplay(){
	
	// Convert Map to Mat (RGB)
	cv::Mat map_mat = cv::Mat(map_.size_y, map_.size_x, CV_64F, map_.prob);
	cv::Mat map_rgb_mat = cv::Mat(map_.size_y, map_.size_x, CV_8UC3);
	map_mat.convertTo(map_mat, CV_32F);
	cv::cvtColor(map_mat, map_rgb_mat, CV_GRAY2RGB, 3);  		

	// Draw Particles
	for (int i=0; i<particles_.size(); i++){
		circle(map_rgb_mat, cv::Point(particles_[i].GetPose().x/10, 
			particles_[i].GetPose().y/10), 4, cv::Scalar(0,255,0), 4);
	}

	// Display Image
	cv::imshow("ParticleFilter", map_rgb_mat);
	cv::waitKey(0);
}
