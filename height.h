#ifndef HEIGHT_H_
#define HEIGHT_H_

#include <iostream>
#include <list>

#include "slam.h"

using namespace std;

#define start_index 256
#define end_index 512
#define reading_count_height 257
#define start_angle pi/4
#define end_angle 3*pi/4
#define angle_per_index 2*pi/1024

class height {

private:
	double field_of_view;
	double h;
	double a, b;
	int valid_count;

	line ground;
	list <line> detected_line_local;
	double data[reading_count_height];
	double bearing_mat[reading_count_height];
	double cos_bearing_mat[reading_count_height];
	double sin_bearing_mat[reading_count_height];
	point points[reading_count_height];

	int m, n; // Size of the input data

public:
	int height_flag;
	void init();
	void slam_points(long int data[]);
	void split_merge(point detected_points[], int n, int threshold);
	double right_distance(point, point, point);
	int line_detection();
	double localization(long int input[]);
	int line_match();
	void L2B(double abc[3], double l[3], double b[3], double offset[3]);
};

#endif
