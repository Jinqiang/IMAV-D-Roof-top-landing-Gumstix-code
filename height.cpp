#include <iostream>
#include <stdio.h>
//#include <math.h>
#include <stdlib.h>
#include <list>

#include "height.h"
#include "state.h"
#include "uav.h"

extern clsState _state;

using namespace std;

void height::init()
{
	valid_count = 0;

    for (int i=0; i<reading_count_height; i++)
    {
		bearing_mat[i] = start_angle + i*angle_per_index;
		if(bearing_mat[i] > pi)
			bearing_mat[i] = bearing_mat[i]-2*pi;
		cos_bearing_mat[i] = cos(bearing_mat[i]);
		sin_bearing_mat[i] = sin(bearing_mat[i]);
    }
}

double height::localization(long int input[])
{
	UAVSTATE state = _state.GetState();
	a = state.a;
	b = state.b;

	slam_points(input);
	line_detection();
	line_match();

//	double pl[3] = {0}; // 3D point in the laser scanner frame
//	double pb[3] = {0}; // 3D point in the UAV body frame
//	double pg[3] = {0}; // 3D point in the heading frame
//
//	pl[0] = ground.r * cos(ground.theta) * 0.001;
//	pl[1] = ground.r * sin(ground.theta) * 0.001;
//	pl[2] = 0;
//
//	double abc[3] = {a, b, 0};
//	double offset[3] = {0.13, 0, 0.08};
//
//	L2B(abc, pl, pb, offset);
//	B2G(abc, pb, pg);
//
//	h = -pg[2];

	h = -ground.r * 0.001;
	return h;
}

void height::slam_points(long int data[])
{
	for (int i=0; i<reading_count_height; i++)
	{
		points[i].r = 0;
		points[i].x = 0;
		points[i].y = 0;
		points[i].theta = 0;
	}

	int k = 0;
    for (int j=0; j<=reading_count_height; j++)
    {
    	int i = j + start_index;
    	if (data[i] >= 20 && data[i] <= 4000)
    	{
			points[k].x = data[i] * cos_bearing_mat[j];
			points[k].y = data[i] * sin_bearing_mat[j];
			points[k].r = data[i];
			points[k].theta = bearing_mat[j];
			k++;
    	}
    }
    valid_count = k;
}

int height::line_detection()
{
    int this_index = 0;
    detected_line_local.clear();
    for (int i=0; i<valid_count-1; i++)
    {
        if (fabs(points[i+1].r-points[i].r)>200 || i==valid_count-2)
        {
			int size = i-this_index+1;
			if (size > 50)
			{
				split_merge(&points[this_index], size, 100);
			}
            this_index = i+1;
        }
    }

    return detected_line_local.size();
}

int height::line_match()
{
	line temp;
	list<line> lines;
    list<line> lines2;

	list<line>::iterator iter1;
	list<line>::iterator iter2;

	// Filter out lines with obvious wrong gradient
    for (iter1=detected_line_local.begin(); iter1!=detected_line_local.end(); iter1++)
	{
        if (fabs(iter1->theta - pi/2) < 0.3 )
        {
			lines.push_back(*iter1);
        }
	}

    // Sort lines according to their perpendicular distance to the origin
    for (iter1=lines.begin(); iter1!=lines.end(); iter1++)
    {
		for (iter2=iter1; iter2!=lines.end(); iter2++)
		{
			if (iter1->r < iter2->r)
			{
				temp = *iter1;
				*iter1 = *iter2;
				*iter2 = temp;
			}
		}
    }

    // Only keep those near to the furthest line
	double furthest = lines.begin()->r;
	for (iter1=lines.begin(); iter1!=lines.end(); iter1++)
	{
		if (fabs(iter1->r-furthest) < 250)
		{
			lines2.push_back(*iter1);
		}
	}

	// Sort the remaining lines by their length
	for (iter1=lines2.begin(); iter1!=lines2.end(); iter1++)
	{
		for (iter2=iter1; iter2!=lines2.end(); iter2++)
		{
			if(iter1->length < iter2->length)
			{
				temp = *iter1;
				*iter1 = *iter2;
				*iter2 = temp;
			}
		}
	}

	iter1 = lines2.begin();

	if (iter1 == lines2.end())
	{
		height_flag = 0;
		return 0;
	}

	else
	    height_flag = 1;

//	cout << "Theta, length, distance: " << iter1->theta*180/pi << ", "
//		 << iter1->length << ", " << iter1->r << endl;

	ground = *iter1;
	return 1;
}

void height::split_merge(point detected_points[], int n , int threshold)
{
    int last = n-1;
    double theta, r;
    double l;
    line detect;

    double x1, y1, x2, y2, x3, y3, k;
    x1 = detected_points[0].x;
    y1 = detected_points[0].y;
    x2 = detected_points[last].x;
    y2 = detected_points[last].y;

    k = (x1*x1+y1*y1-x1*x2-y1*y2) / (x1*x1+x2*x2+y1*y1+y2*y2-2*x1*x2-2*y1*y2);
    x3 = x1 + k*(x2-x1);
    y3 = y1 + k*(y2-y1);

    r = sqrt(x3*x3+y3*y3);
    theta = atan2(y3, x3);

    double point_distance = 0;
    double winner_value = 0;
    int winner_index = 0;

    for (int j=1; j<last; j=j+6)
    {
        point_distance = right_distance(detected_points[0], detected_points[last], detected_points[j]);
        if (point_distance > winner_value)
        {
            winner_value = fabs(point_distance);
            winner_index = j;
        }
    }
    if (winner_value > threshold)
    {
        split_merge(detected_points, winner_index+1, threshold);
        split_merge(&detected_points[winner_index], n-winner_index, threshold);
    }
    else
    {
        l = (detected_points[last].x- detected_points[0].x)*(detected_points[last].x-detected_points[0].x);
        l+= (detected_points[last].y- detected_points[0].y)*(detected_points[last].y-detected_points[0].y);
        l = sqrt(l);
        detect.p1 = detected_points[0];
        detect.p2 = detected_points[last];
        detect.theta = theta;
        detect.r = r;
        detect.length = l;
        detected_line_local.push_back(detect);
    }
}

// Calculate perpendicular distance from point c to line ab
double height::right_distance(point line_a, point line_b, point c)
{
	double x1, y1, x2, y2, x3, y3, d;
	x1 = line_a.x;
	y1 = line_a.y;
	x2 = line_b.x;
	y2 = line_b.y;
	x3 = c.x;
	y3 = c.y;

	d = fabs( ((x3-x1)*(y2-y1)-(y3-y1)*(x2-x1)) / sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)) );
	return d;
}

void height::L2B(double abc[3], double l[3], double b[3], double offset[3])
{
	b[0] = -l[0] + offset[0];
	b[1] = l[2] + offset[1];
	b[2] = l[0] + offset[2];
}
