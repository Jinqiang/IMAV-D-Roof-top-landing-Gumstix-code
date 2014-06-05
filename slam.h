#ifndef SLAM_H_
#define SLAM_H_

#include <iostream>
#include <list>

using namespace std;

#define pi 3.141592654
#define reading_count 1001//1081
#define known_feature_size 10
#define each_angle 0.25*pi/180.0

struct point {
	double x;
	double y;
	double theta;
	double r;
	int index;
	double distance;
};

struct line {
	point p1;
	point p2;
	double theta;
	double r;
	double length;
	double covariance;
	int index;
};

struct position {
	double x;
	double y;
	double theta;
	double height;
	int flag;
};

class slam {

private:
	double field_of_view;
	position known_features[10];
	position pre_position;
	position cur_position;
	list <line> detected_line_local;
	double data[reading_count];
	point points[reading_count];
	double bearing_mat[reading_count];
	double cos_bearing_mat[reading_count];
	double sin_bearing_mat[reading_count];
	list<point> matched_global_feature;
	int valid_count;
	double circle_left;
	double circle_right;

public:
	void init();
	void slam_points(double data[]);
	void range_f(long int input[], double data[]);
	void split_merge (point detected_points[], int n, int threshold);
	double right_distance(point, point, point);
	point B2G2D(point, double, double);
	int line_detection();
	position localization(long int input[]);
	int point_match();
	void line_f(int threshold);
	void feature_matching(point p, int threshold);
	void point_local(point close1, point close2);
	void point_local(point close1);
	double mod(double x, double y);
	void update_angle();
};

#endif
