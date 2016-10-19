#ifndef DATA_STRUCTS_H
#define DATA_STRUCTS_H

#include <Eigen/Dense>

/** Useful data structures */

// Pose
typedef struct {
	double x, y, theta; 
} Pose;

// Laser range data
typedef struct {
	int rows, cols;
	Eigen::MatrixXd data;
} LaserData;

// Odometry data
typedef struct {
	int rows, cols;
	Eigen::MatrixXd data;
} OdomData;

#endif
