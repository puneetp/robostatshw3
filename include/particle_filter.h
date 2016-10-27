#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include "particle.h"
#include "data_structs.h"
#include "Eigen/Dense"
#include <cv.h>

extern "C" {
	#include "bee-map.h"
}

/** Defines the Particle Filter class */

class ParticleFilter {
public:
	/** Constructor */
	ParticleFilter(int num_particles, double motion_mean, double motion_sigma,
	double laser_mean, double laser_sigma, char* data_file, char *map_file);

	/** Reads laser, odom and map data */
	void ReadData(char* data_file, char *map_file);

	/** Particle filter algorithm.
	* @return  trajectory: the robot's trajectory
	*/
	void Filter(std::vector<Pose> &trajectory);

	void Filter_new(std::vector<Pose> &trajectory); 

	/************************ Test code ****************************************/
	void DumpParticlesToFile();
	void DumpOdomToFile();
	void DumpLaserToFile();
	void TestMotionModel();

	/* ****************** Member variables ********************************* */
	int num_particles_;
	std::vector<Particle> particles_;
	map_type map_;
	LaserData laser_data_;
	OdomData odom_data_;
	double motion_mean_, motion_sigma_;  			// gaussian parameters for motion model
	double laser_mean_, laser_sigma_;				// gaussian parameters for laser models
	std::vector<MapCell> unoccupied_list_;			// List of map cells known to be empty with certainty 
	cv::Mat img_;
	cv::Mat img_map_;
	/* ********************** Member functions ***************************** */

	/** Initializes particles on the map */
	void InitParticles();

	void HackInitParticles();

	/** Preprocesses the map. Builds a list of cells with occupancy probability = 0 **/
	void PreprocessMap();

	/** Updates the position of particle p given previous and current odom readings */ 
	void MotionModel(Particle &p, Eigen::Matrix3d T1, Eigen::Matrix3d T2);

	/** Prob Rob Book Motion Model*/
	void MotionModel(Particle &p, Pose p1, Pose p2);

	/** Computes the transformation matrix given x, y theta. Used by the motion model. */
	Eigen::Matrix3d ComputeTransform(double x, double y, double theta);

	/** Returns a "noisy" transform matrix between two poses.
	* Used by the motion model to update a particle's pose.
	*/
	Eigen::Matrix3d NoisyTransform(Eigen::Matrix3d T1, Eigen::Matrix3d T2);


	/**Sensor mode is core fuction which calculates the weights of the particles
	* it takes in information of particle location , the map of the world and the laser reading at that place
	* this allows it to caclulate the probability of getting a reading given robots position . Returns weight parameter
	*/
	double SensorModel( Particle &p , int laser_index);


	/** create PDF for the sensor model*/
	void Sensor_models_laser_PDF_vector( double map_directed_obstacle_range[] ,int hop, int laser_index, double per_particle_sensor_probability_vector[] );


	/* Returns the measurement probability for a single laser ray given position. Used by the sensor model.
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

	/** Regenerates the particles_ according to weights */
	void ImportanceSampling(std::vector<Particle> &particles, int verbose=0);

	/** Updates the visualization of the map */
	void UpdateDisplay();

	/** Gets x/y coordinates from row/col. ASSUMES XY ORIGIN IS AT BOTTOM LEFT CORNER,
	* and array indexing starts from the top left corner. Returns X/Y coords of the cell's center.
	**/
	void GetXYFromIndex(double &x, double &y, int row, int col);

	/** Gets row/col index from X/Y coords. ASSUMES XY ORIGIN IS AT BOTTOM LEFT CORNER,
	* and array indexing starts from the top left corner.
	**/
	void GetIndexFromXY(double x, double y, int &row, int &col);
	
	void DrawMap();

	void DrawAllParticles();

	void DrawRay(double x, double y, double x1, double y1);
	
	void DrawParticle(Particle particle);
};

#endif
