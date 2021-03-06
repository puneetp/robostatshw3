#ifndef DATA_STRUCTS_H
#define DATA_STRUCTS_H

#include "Eigen/Dense"

/** Useful data structures */

// Pose
typedef struct {
	double x, y, theta; 
} Pose;

// Laser range data
typedef struct {
	int rows;
	Eigen::MatrixXd data;
} LaserData;

// Odometry data
typedef struct {
	int rows;
	Eigen::MatrixXd data;
} OdomData;

// Map cell
typedef struct {
	int row, col;
} MapCell;

#endif
