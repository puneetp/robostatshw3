#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include "particle.h"
#include "data_structs.h"
#include "Eigen/Dense"

extern "C" {
	#include "bee-map.h"
}

/** Defines the Particle Filter class */

class ParticleFilter {
public:
	/** Constructor */
	ParticleFilter(int num_particles, double motion_mean, double motion_sigma,
		double laser_mean, double laser_sigma) {
		
		num_particles_ = num_particles;
		motion_mean_ = motion_mean;
		motion_sigma_ = motion_sigma;
		laser_mean_ = laser_mean;
		laser_sigma_ = laser_sigma;
	}

	/** Reads laser, odom and map data */
	void ReadData(char* data_file, char *map_file);

	/** Particle filter algorithm.
	* @return  trajectory: the robot's trajectory
	*/
	void Filter(std::vector<Pose> &trajectory);

private:
	/* ****************** Member variables ********************************* */
	int num_particles_;
	std::vector<Particle> particles_;
	map_type map_;
	LaserData laser_data_;
	OdomData odom_data_;
	double motion_mean_, motion_sigma_;  			// gaussian parameters for motion model
	double laser_mean_, laser_sigma_;				// gaussian parameters for laser model

	/* ********************** Member functions ***************************** */

	/** Initializes particles on the map */
	void InitParticles();

	/** Updates the position of particle p given previous and current odom readings */ 
	void MotionModel(Particle &p, Eigen::Matrix3d T1, Eigen::Matrix3d T2);

	/** Computes the transformation matrix given x, y theta. Used by the motion model. */
	Eigen::Matrix3d ComputeTransform(double x, double y, double theta);

	/** Returns a "noisy" transform matrix between two poses.
	* Used by the motion model to update a particle's pose.
	*/
	Eigen::Matrix3d NoisyTransform(Eigen::Matrix3d T1, Eigen::Matrix3d T2);

	/** Updates the weight of particle p given laser readings.
	* @param laser_idx row index of current measurement in laser data.
	*/
	void SensorModel(Particle &p, int laser_idx);

	/** Returns the measurement probability for a single laser ray given position. Used by the sensor model.
	* Constructs an appropriate probability distribution for the given sensing modality.
	* @param range_measurement actual range value for the given ray_angle from laser data log
	*/
	double MeasurementProb(double range_measurement, double ray_angle, Pose pos);

	/** Returns the simulated range sensed by the laser given a particle's pose and the ray angle.
	* Used by the sensor model.
	*/
	double GetRangeFromMap(double ray_angle, Pose p);

	/** Implements importance sampling to re-sample particles */
	void ResampleParticles();
};

#endif