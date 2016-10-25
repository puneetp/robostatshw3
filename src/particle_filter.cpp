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

#define NUM_ODOM 							4
#define START_LASER_INDEX   				6
#define NUM_LASER_VALS						180
#define NUM_LASER							187
#define LASER_POD_FORWARD_OFFSET   			25 // This is in 
#define UNOCCUPIED_TOL						1e-2

// Laser params
#define THRESHOLD_FOR_OBSTACLE 				0.9
#define MIN_PROBABILITY_VALUE				0.1
#define LASER_HOP 							4 // How many lasers do we want to hop in search space Minimum is one
#define EXP_MULTIPLIER						1.0
#define GAUSSIAN_MULTIPLIER					70
#define AT_WORLDS_END						2000
#define EOR_PROB  							0.3	
#define RANGE_INCREMENT						5

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
	std::random_device rd;
	std::default_random_engine generator(rd());
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

	fclose(f);

	// Read map
	read_beesoft_map(map_file, &map_);

	std::cout << "Num odom entries: " << odom_data_.rows << "\n";
	std::cout << "Num laser entries: " << laser_data_.rows << "\n";

	// dump map
	FILE *f1 = fopen("map.csv", "w");
	for(int i = 0; i < map_.size_x; ++i) {
		for(int j = 0; j < map_.size_y-1; ++j) {
			fprintf(f1, "%lf,", map_.prob[i][j]);
		}
		fprintf(f1, "%lf\n", map_.prob[i][map_.size_y-1]);
	}
	fclose(f1);
}

//

void ParticleFilter::
Filter(std::vector<Pose> &trajectory) {
	int laser_idx(0), odom_idx(0);
	Eigen::Matrix3d prev_T, curr_T;

	// Preprocess map
	PreprocessMap();

	// Initialize particles
	// InitParticles();
	HackInitParticles();

	UpdateDisplay();

	// DumpParticlesToFile();
	// DumpOdomToFile();
	// DumpLaserToFile();

	// UpdateDisplay();

	// // Initialize with first odom entry
	prev_T = ComputeTransform(laser_data_.data(0, 0), laser_data_.data(0, 1), laser_data_.data(0, 2));
	// ++odom_idx;
	++laser_idx;

	while(laser_idx < 100)
	{
		std::cout << "Laser idx: " << laser_idx << "\n";
		// Read log file. Let's stick with laser for now?
		curr_T = ComputeTransform(laser_data_.data(laser_idx, 0), laser_data_.data(laser_idx, 1), laser_data_.data(laser_idx, 2));
		// Apply motion and sensor model on all particles
	 	for(int i = 0; i < num_particles_; ++i) {
			// std::cout << "Paticle number: " << i << "\n";

			// motion model
			MotionModel(particles_[i], prev_T, curr_T);
			// std::cout << "Motion model done\n";

			// sensor model
	 		SensorModel(particles_[i], laser_idx);
	 		// std::cout << "Sensor model done\n";
	 		//std::cout << " num_particles_ i=" <<i<<std::endl;
	 	}
		// Importance sampling
	 	ImportanceSampling(particles_);
	 	// std::cout << "Importance sampling done\n";

	 	prev_T << curr_T;

	 	// Display
	 	UpdateDisplay();

	 	laser_idx++;		
	}
}

/* Regenerates the particles_ according to weights
 */
void ParticleFilter::ImportanceSampling(std::vector<Particle> &particles , int verbose){
	
	// Compute vector of cumulative weights
	const unsigned int num_particles = particles.size();
	std::vector<double> cum_weights(num_particles+1, 0.0);
	for (int i=1; i<num_particles+1; i++){
		cum_weights[i] = cum_weights[i-1] + std::fabs(particles[i-1].GetWeight());
	}

	// Generate new particles
	std::vector<Particle> new_particles;
	std::random_device rd;
	std::default_random_engine generator(rd());
	std::uniform_real_distribution<float> distribution(0.0, cum_weights[num_particles]);

	std::cout << "cum weights: " << cum_weights[num_particles] << "\n";
	
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
		int index = lower - cum_weights.begin() - 1; // -1 => caz weight has more 1 more element
		if(index < 0) {
			continue;
		}
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
   //Check if you are in gray space , if yes return 0 weight
   //For the values X and Y and Tetha project Laser out into the MAP.
   //Read the MAP in these laser rays and get back values of first obstruction as per the map,
   //...rest weight out the gray values if they come before.
   //Use the value to a) Map Reading of lasters b) Laser reading to fetch value c) The value fetched call this in loop NUM_LASER times
   //Return the sum of log of all the probabilties ( CHECK again here for what calculation has to be done.)
	 
	double map_directed_obstacle_range[NUM_LASER_VALS];
	

	//Initializing
	int search_increment = RANGE_INCREMENT;

	int hop=LASER_HOP;// How many lasers do we want to hop in search space Minimum is one

	//initatied to -1 , if -1 don't use the value 
	std::fill_n(map_directed_obstacle_range, NUM_LASER_VALS, -1); // to -1 

	//Running accross all lasers 	
	//for ( int i =0 ; i<NUM_LASER;i=i+hop)
	for ( int i2 =0 ; i2<NUM_LASER_VALS ;i2=i2+hop) //NUM_LASER-START_LASER_INDEX
	{
		int x=0;  // col
		int y=0;  // row		
		int r=0;
		bool reached(false);
		while (  x<map_.size_x && y<map_.size_y  &&  x >= 0 && y >= 0 && !reached  )  
		{
			//std::cout << " value of pi" <<M_PI << std::endl;

			double increment_scan_angle_phi = i2*M_PI/180.0; // Angle of laser from start

			double start_offset_laser = -90.0*M_PI/180.0; // starting point -90 degree

			//std::cout << " value of increment_scan_angle_phi is= "<<increment_scan_angle_phi<< " value of start_offset_laser is " <<start_offset_laser << std::endl;

			double robot_theta= p.GetPose().theta;		

			double rx=r*std::cos(robot_theta + increment_scan_angle_phi + start_offset_laser);

			double ry=r*std::sin(robot_theta + increment_scan_angle_phi+start_offset_laser);

			//std::cout << " value of robot_theta + increment_scan_angle_phi+start_offset_laser = "<<robot_theta + increment_scan_angle_phi+start_offset_laser<< std::endl;
			
			double laser_x_offset = LASER_POD_FORWARD_OFFSET*std::cos(robot_theta);

			double laser_y_offset = LASER_POD_FORWARD_OFFSET*std::sin(robot_theta);

			// x= std::floor((p.GetPose().x + rx + laser_x_offset) / 10.0);

			// y= std::floor((p.GetPose().y + ry + laser_y_offset) / 10.0);

			GetIndexFromXY(p.GetPose().x + rx + laser_x_offset, 
				p.GetPose().y + ry + laser_y_offset,
				y, x);


			if ( ((x<map_.size_x) && (y<map_.size_y))  &&  ((x >= 0) && ( y >= 0)) )
			{

				double obstacle_prob=map_.prob[y][x];						
				
				//std::cout << " value of x is= "<<x<< " value of y is " <<y << " obstacle_prob " << obstacle_prob<<std::endl;

				if (obstacle_prob > THRESHOLD_FOR_OBSTACLE) // Note we check for certain threshold
				{
					map_directed_obstacle_range[i2]=r;				
					// std::cout <<"get p.GetPose().x=" <<p.GetPose().x<<" x="<<x<<" y="<<y<< " value of p.GetPose().y="<<p.GetPose().y << " i2=" <<i2 << " r="<<r<<" value of x is= "<<x<< " y=" <<y << " obstacle_prob=" << obstacle_prob<<std::endl;
					reached=true; // check this command
				}
				else if(obstacle_prob == -1) {
					// Stop searching when we hit unknown space, and discard this laser angle
					reached = true;
				}

			}

			r=r+search_increment;
			//std::cout << "r "<<r ;
		}
	}

	double per_particle_sensor_probability_vector[NUM_LASER_VALS];

	std::fill_n(per_particle_sensor_probability_vector, NUM_LASER_VALS, -1); // to -1 


	//Call the function to generate laser probabilities per particle.	
    Sensor_models_laser_PDF_vector( map_directed_obstacle_range, hop, laser_row_index, per_particle_sensor_probability_vector );

    double weight_log = 0.0;

	for (int j =0 ; j < NUM_LASER_VALS ; j=j+hop)
	{
		if (!(std::abs(per_particle_sensor_probability_vector[j] - (-1)) < 1e-4)) {
		// if(per_particle_sensor_probability_vector[j] != -1) {

			weight_log += std::log(per_particle_sensor_probability_vector[j]);
		}
		//std::cout <<" weight_log " <<weight_log<< "  per_particle_sensor_probability_vector[j]" <<per_particle_sensor_probability_vector[j] <<std::endl;

	}

	double weight_value= std::exp(weight_log);  //actual weight

	//std::cout <<"weight_log total "<< weight_log <<" final weight_value "<<weight_value <<std::endl;

	// Update particle's weight
	p.SetWeight(weight_value);

	return weight_value;

}


// Matlab equivalent is one line plot /(x,max(exp(-x),normpdf(x,9,.5))) // produce medium peaky graphs

void ParticleFilter::Sensor_models_laser_PDF_vector( double map_directed_obstacle_range[],int hop,int laser_row_index,double per_particle_sensor_probability_vector[] )
{	

	double laser_mean;
	double laser_std=laser_sigma_; //editable
	double exp_value=0.0;
	double normal_value=0.0;
	double eor_prob = 0.0;
	
	for( int i =0 ; i < NUM_LASER_VALS ; i=i+hop )

	{
		//std::cout <<" Sensor_models_laser_PDF_vector i="<<i<<std::endl;

		if (!(std::abs(map_directed_obstacle_range[i] - (-1)) < 1e-4))
		// if(map_directed_obstacle_range[i] != -1)
		{
			//create normal distribution
			laser_mean=map_directed_obstacle_range[i];

			// fetch reading at value
			double fetch_laser_value_at_this_point = laser_data_.data(laser_row_index, i+START_LASER_INDEX); // REVIEW this

			//create normal distribution
			// std::normal_distribution<double> distribution (laser_mean, laser_std);

			// Get probability from Normal Distrubution at this point
			double normal_value1st_half = (1.0/(laser_std*std::sqrt(2*M_PI))) ;

			double normal_value2nd_half = std::exp( -0.5 * std::pow( (fetch_laser_value_at_this_point - laser_mean )/laser_std, 2.0)   )      ;

			normal_value = GAUSSIAN_MULTIPLIER * normal_value1st_half* normal_value2nd_half ;	

			// From Exponential Function of PDF , since its falling , it e^(-x)
			exp_value=std::exp(-EXP_MULTIPLIER * fetch_laser_value_at_this_point);

			// End of range magic
			if(fetch_laser_value_at_this_point > AT_WORLDS_END) {
				eor_prob = EOR_PROB;
			}
			else {
				eor_prob = 0.0;
			}

			// add some minor noise and get max of two
			double max_step1=std::max(exp_value,MIN_PROBABILITY_VALUE);
			
			// get the maximum of the two
			double val=std::max(max_step1,normal_value);

			per_particle_sensor_probability_vector[i]= std::max(val, eor_prob);

			// if (!(i%10))
			// 	std::cout <<" i="<<i<<" laser_mean="<< laser_mean << " fetch_laser_value_at_this_point=" << fetch_laser_value_at_this_point<< " normal_value=" <<normal_value   <<" n1="<<normal_value1st_half<<" n2="<<normal_value2nd_half << " exp_value=" <<exp_value<<" max_step1=" <<max_step1<<" per_particle_sensor_probability_vector[]=" <<per_particle_sensor_probability_vector[i] << std::endl;

		}
	}

}

void ParticleFilter::
PreprocessMap() {
	// iterate through the map and find unoccupied cells
	for(int i = 0; i < map_.size_x; ++i) {
		for(int j = 0; j < map_.size_y; ++j) {
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
	std::random_device rd;
	std::default_random_engine generator(rd());
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
		// x = unoccupied_list_[cell_idx].col * map_.resolution + map_.resolution/2;
		// y = unoccupied_list_[cell_idx].row * map_.resolution + map_.resolution/2;
		GetXYFromIndex(x, y, unoccupied_list_[cell_idx].row, unoccupied_list_[cell_idx].col);

		p.SetPose(x, y, theta);
		p.SetT(ComputeTransform(x, y, theta));

		if(i == 0) {
			std::cout << x << ", " << y << ", " << theta << "\n";
		}

		particles_.push_back(p);
	}
}

void ParticleFilter::
HackInitParticles() {
	int seed_row(400), seed_col(440), num_thetas(20);
	int window_size(30);
	double x, y, theta;
	num_particles_ = 0;

	std::random_device rd;
	std::default_random_engine generator(rd());
	std::uniform_real_distribution<double> real_distribution(0.0, 2*M_PI);

	for(int i = seed_row - window_size; i < seed_row + window_size; i += 2) {
		for(int j = seed_col - window_size; j < seed_col + window_size; j += 2) {
			for(int k = 1; k <= num_thetas; ++k) {
				Particle p;
				// y = i * map_.resolution + map_.resolution/2;
				// x = j * map_.resolution + map_.resolution/2;
				GetXYFromIndex(x, y, i, j);
				theta = real_distribution(generator);

				p.SetPose(x, y, theta);
				p.SetT(ComputeTransform(x, y, theta));

				particles_.push_back(p);
				++num_particles_;
			}
		}
	}

	std::cout << "num particles: " << num_particles_ << "\n";
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
		// UpdateDisplay();
	}

	fclose(f);
}

/** Updates the visualization of the map */
void ParticleFilter::UpdateDisplay(){
	
	// Convert Map to Mat (RGB)
	// cv::Mat map_mat = cv::Mat(map_.size_x, map_.size_y, CV_64F, map_.prob);
	// cv::Mat map_rgb_mat = cv::Mat(map_.size_x, map_.size_y, CV_8UC3);
	// map_mat.convertTo(map_mat, CV_32F);
	// cv::cvtColor(map_mat, map_rgb_mat, CV_GRAY2RGB, 3);  		
	cv::Mat map_mat = cv::Mat(map_.size_y, map_.size_x, CV_64F, map_.prob);
	cv::Mat map_rgb_mat = cv::Mat(map_.size_y, map_.size_x, CV_8UC3);
	map_mat.convertTo(map_mat, CV_32F);
	cv::cvtColor(map_mat, map_rgb_mat, CV_GRAY2RGB, 3);  
	// cv::transpose(map_rgb_mat, map_rgb_mat);

	// Draw Particles
	int row, col;
	for (int i=0; i<particles_.size(); i++){
		GetIndexFromXY(particles_[i].GetPose().x, particles_[i].GetPose().y, row, col);
		// circle(map_rgb_mat, cv::Point(particles_[i].GetPose().x/10, 
		// 	particles_[i].GetPose().y/10), 2, cv::Scalar(0,255,0), 2);
		circle(map_rgb_mat, cv::Point(col, row), 2, cv::Scalar(0,255,0), 2);
	}

	// Display Image
	cv::imshow("ParticleFilter", map_rgb_mat);
	cv::waitKey(1);
}

void ParticleFilter::
GetXYFromIndex(double &x, double &y, int row, int col) {
	x = col * map_.resolution + map_.resolution/2;
	y = map_.size_y * map_.resolution - (row * map_.resolution + map_.resolution/2);
}

void ParticleFilter::
GetIndexFromXY(double x, double y, int &row, int &col) {
	col = std::floor(x / map_.resolution);
	row = std::floor(map_.size_y - y / map_.resolution);

	// sanitize
	if(col >= map_.size_x) {
		col = map_.size_x - 1;
	}
	if(row >= map_.size_y) {
		row = map_.size_y - 1;
	}
}