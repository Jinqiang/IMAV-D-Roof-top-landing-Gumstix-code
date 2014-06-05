#include <iostream>
#include <stdio.h>
//#include <math.h>
#include <stdlib.h>
#include <list>

#include "slam.h"
#include "uav.h"
#include "state.h"

extern clsState _state;
#define SIZE 1440

using namespace std;

void slam::init()
{
	field_of_view = 270.0*pi/180;
	cur_position.flag = 1;

	/* For TLab */
	pre_position.theta = cur_position.theta =  0;/*-pi/2.0; //*/
	pre_position.x = cur_position.x = -4070; //-2400; //0; //-4260; //0;
	pre_position.y = cur_position.y = -1995; //5000; //-2500; //-2250; //-3500;

	circle_left = -5010;
	circle_right = -3510;

	double feature_x[10] = {-6750, circle_left,  circle_right,     -sqrt(2.0)*1000,    0,    0, 0, 3650, 5150, 6890};
	double feature_y[10] = {    0,           0,             0, 6000+sqrt(2.0)*1000, 8000, 6000, 0,    0,    0,    0};


	/* For SAFMC */
//	pre_position.theta = cur_position.theta =  0;/*-pi/2.0; //*/
//	pre_position.x = cur_position.x = -4500;
//	pre_position.y = cur_position.y = -3500;
//
//	circle_left = -5000;
//	circle_right = -3500;
//
//	double feature_x[10] = {-7000, circle_left,  circle_right, 0,      -sqrt(2.0)*1000,     0,    0, 3500, 5000, 7000};
//	double feature_y[10] = {    0,           0,             0, 0,  6000+sqrt(2.0)*1000,  8000, 6000,    0,    0,    0};

	/* For Engine Audi */
//	pre_position.x = -5300;	pre_position.y = -2400;
//	cur_position.x = -5300;	cur_position.y = -2400;
//	double feature_x[10] = {3047, -3047,  4406, -4406,   5494, -5494, 666666, 777777, 888888, 999999};
//	double feature_y[10] = {   0,     0, -1360, -1360,   -271,  -271, 666666, 777777, 888888, 999999};

    for(int i=0; i<10; i++)
    {
        known_features[i].x = feature_x[i];
        known_features[i].y = feature_y[i];
    }

	for(int i=0; i<reading_count; i++)
	{
		bearing_mat[i] = (pi-field_of_view)/2+i*each_angle;
		if(bearing_mat[i] > pi)
			bearing_mat[i] = bearing_mat[i]-2*pi;
		cos_bearing_mat[i] = cos(bearing_mat[i]);
		sin_bearing_mat[i] = sin(bearing_mat[i]);
	}

	valid_count = 0;
}

position slam::localization(long int input[])
{
    range_f(input, data);
    slam_points(data);

    int line_count = line_detection();

  //  cout << "No. of lines detected: " << line_count << endl;

    if (line_count == 0)
    {
    	cout << "No line detected!" << endl;
    	cur_position.flag = 0;
    }

    else
    {
    	update_angle();
    	UAVSTATE &state = _state.GetState();

    	double h_offset = fabs(state.z - 0.32 + 1.2);
    	double c_offset = 0.0;
    	if (h_offset < 0.5)
    	{
    		c_offset = (0.75 - sqrt(0.75*0.75-h_offset*h_offset))*1000;
    	}
    	else
    		c_offset = 99999.9;

		known_features[1].x = circle_left + c_offset;
		known_features[2].x = circle_right - c_offset;

//		cout << circle_left << " " << circle_right << " " << known_features[1].x << "  " << known_features[2].x << " " << c_offset<<endl;

    	if (point_match())
    		cur_position.flag = 1;
    	else
    	{
    		cur_position.flag = 0;
//    		cout << "Matching fail!" << endl;
    	}
    }

	return cur_position;
}

void computeRange(double x, double y, double range[SIZE])
{
	double xMin = -7500;
	double xMax =  7500;
	double yMin = -4500;
	double yMax = 14500;

	if ((x<xMax)&&(x>xMin)&&(y<yMax)&&(y>yMin))
	{
		double dTheta = (0.25/180.0)*pi;
		double theta[5];
		double d[5] = {xMax-x, yMax-y, x-xMin, y-yMin, xMax-x};

		theta[0] = atan((yMax-y)/(xMax-x));
		theta[1] = atan((x-xMin)/(yMax-y)) + pi/2;
		theta[2] = atan((y-yMin)/(x-xMin)) + pi;
		theta[3] = atan((xMax-x)/(y-yMin)) + pi/2*3;
		theta[4] = 3*pi;

		int index = 0;
		int thetaIndex = 0;
		for (double angle=0; angle<=(2.0*pi); angle+=dTheta)
		{
			if (angle > theta[thetaIndex])
				thetaIndex = thetaIndex + 1;

			range[index] = fabs(d[thetaIndex]/cos(angle-pi/2*(thetaIndex)));

			//printf("{(%3.4lf %3.4lf),(%d %d)} ", angle, theta[thetaIndex],index, thetaIndex);
			index = index+1;
		}
	}
	else
	{
		cout << "Out of bound!" << endl;
		double angle;
		int index = 0;
		double dTheta = (0.25/180.0)*pi;
		for ( angle = 0; angle <=(2.0*pi); angle +=dTheta)
		{
			range[index] = 20000;
			index++;
		}
    }
}

void slam::range_f(long int input[], double data[])
{
    double range[SIZE] = {0};
    double x = cur_position.x;
    double y = cur_position.y;
    double angle = cur_position.theta;

    computeRange(x, y, range);

    int startIndex =(int)((angle-(45.0/180*pi)+2*pi)/(0.25/180*pi));

	for (int i=0; i<reading_count; i++)
	{
		int rangeIndex = (i+startIndex) % SIZE;

		if (input[i] > (int)range[rangeIndex])
			data[i] = 0;
		else
			data[i] = input[i];
//		data[i] = input[i];
	}
}

//void slam::range_f(long int input[],double data[])
//{
//    for (int i=0; i<reading_count; i++)
//    {
////        if (bearing_mat[i]<0 || bearing_mat[i]>pi)
////            data[i]=0;
////        else
//            data[i]=input[i];
//    }
//}

void slam::slam_points(double data[])
{
	for (int i=0; i<reading_count; i++)
	{
		points[i].r = 0;
		points[i].x = 0;
		points[i].y = 0;
		points[i].theta = 0;
		points[i].index = 0;
	}

	int k = 0;
    for (int i=0; i<reading_count; i++)
    {
    	if (data[i] >= 20 && data[i] < 30000)
    	{
			points[k].x = data[i]*cos_bearing_mat[i];
			points[k].y = data[i]*sin_bearing_mat[i];
			points[k].theta = bearing_mat[i];
			points[k].r = data[i];
			points[k].index = k;
			k++;
    	}
    }
    valid_count = k;
}

int slam::line_detection()
{
    int start_index = 0;
    detected_line_local.clear();
    for (int i=0; i<valid_count-1; i++)
    {
    	double distance = (points[i+1].x - points[i].x)*(points[i+1].x - points[i].x)
    			+ (points[i+1].y - points[i].y)*(points[i+1].y - points[i].y);
//        if (fabs(points[i+1].r-points[i].r)>400 || i==valid_count-2)
        if (distance>(400.0*400.0) || i==valid_count-2)
        {
			int size = i-start_index+1;
			if (size > 25)
			{
				split_merge(&points[start_index], size, 100);
			}
            start_index = i+1;
        }
    }

    line_f(500);
    return detected_line_local.size();
}

void slam::split_merge(point detected_points[], int n , int threshold)
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
double slam::right_distance(point line_a, point line_b, point c)
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

point slam::B2G2D(point in, double phi, double tht)
{
	point out = in;
	out.x = cos(tht)*in.x + sin(tht)*sin(phi)*in.y;
	out.y = cos(phi)*in.y;

	return out;
}

void slam::line_f(int threshold)
{
    list<line>::iterator l = detected_line_local.begin();
    for( ; l!=detected_line_local.end(); )
    {
        if(l->length < threshold)
        {
        	list<line>::iterator temp_l = l;
        	l++;
            detected_line_local.erase(temp_l);
        }
        else
            l++;
    }
}

int slam::point_match()
{
    point temp;
    point close1, close2;

    matched_global_feature.clear();
    cur_position = pre_position;

    list<line>::iterator iter = detected_line_local.begin();
//    cout << "No. of detected lines: " << detected_line_local.size() << endl;
//    cout << "1st line length: " << (*iter).length << endl;
//    cout << "1st line angle: " << (*iter).theta*180/pi << endl;

	for( ; iter!=detected_line_local.end(); iter++)
	{
		// End points not counted as features

		// For the case of Q1-Lion
		double start_limit = -pi/4.0;
		double end_limit = -3.0*pi/4.0;

		// For the case of Q2-Lion
		if (reading_count != 1081)
		{
			end_limit = -155.0/180.0*pi;
		}

		if (fabs((*iter).p1.theta-end_limit)>0.1 && fabs((*iter).p1.theta-start_limit)>0.1)
			feature_matching((*iter).p1, 500000);

		if (fabs((*iter).p2.theta-end_limit)>0.1 && fabs((*iter).p2.theta-start_limit)>0.1)
			feature_matching((*iter).p2, 500000);
	}

	static int count = 0;
	count ++;
	if(count > 1e7) count = 0;
    if(count % 50 == 0){
    	cout << "No. of matched features: " << matched_global_feature.size() << endl;
    }

    if (matched_global_feature.size()==0)
	   return 0;

	list<point>::iterator iter1 = matched_global_feature.begin();
	list<point>::iterator iter2;
	for( ; iter1!=matched_global_feature.end(); iter1++)
	{
		iter2 = iter1;
		for( ; iter2!=matched_global_feature.end(); iter2++)
		{
//			if (iter1->distance > iter2->distance)
			if (iter1->r > iter2->r)
			{
				temp = *iter1;
				*iter1 = *iter2;
				*iter2 = temp;
			}
		}
	}

//    iter1 = matched_global_feature.begin();
//    for( ; iter1!=matched_global_feature.end(); iter1++)
//    {
//    	cout<<"Matched feature index "<<iter1->index<<" ";
//    	cout<<"("<<iter1->x<<", "<<iter1->y<<")"<<" "<<sqrt(iter1->distance)<<endl;
//
//    }
    
    iter1 = matched_global_feature.begin(); 
    close1 = *iter1; // Closest feature 1

    if (matched_global_feature.size()==1)
    {
    	point_local(close1);
    	pre_position = cur_position;
    	return 1;
    }

    iter1++;

    while ( (*iter1).index==close1.index && iter1!=matched_global_feature.end() )
    {
    	iter1++;
    }

    if (iter1==matched_global_feature.end())
    {
    	point_local(close1);
    	pre_position = cur_position;
    	return 1;
    }
  
    close2 = *iter1; // Close feature 2
//  cout<<"Close feature 1"<<"("<<close1.index<<close1.x<<","<<close1.y<<")"<<endl;
//	cout<<"Close feature 2"<<"("<<close2.index<<close2.x<<","<<close2.y<<")"<<endl;
    point_local(close1, close2);
    pre_position = cur_position;
    return 1;
}

void slam::point_local(point close1)
{
	double theta_temp1 = close1.theta + cur_position.theta;

    double xtemp1 = known_features[close1.index].x - close1.r*cos(theta_temp1);
	double ytemp1 = known_features[close1.index].y - close1.r*sin(theta_temp1);

	cur_position.x = xtemp1;
	cur_position.y = ytemp1;
}

void slam::point_local(point close1, point close2)
{
	double theta_temp1 = close1.theta + cur_position.theta;
	double theta_temp2 = close2.theta + cur_position.theta;

    double xtemp1 = known_features[close1.index].x - close1.r*cos(theta_temp1);
	double ytemp1 = known_features[close1.index].y - close1.r*sin(theta_temp1);

    double xtemp2 = known_features[close2.index].x - close2.r*cos(theta_temp2);
	double ytemp2 = known_features[close2.index].y - close2.r*sin(theta_temp2);

	cur_position.x = (xtemp1+xtemp2) / 2;
	cur_position.y = (ytemp1+ytemp2) / 2;
}

void slam::feature_matching(point p, int threshold)
{
    point global_feature;

    global_feature.x = p.x*cos(cur_position.theta) - p.y*sin(cur_position.theta) + cur_position.x;
    global_feature.y = p.x*sin(cur_position.theta) + p.y*cos(cur_position.theta) + cur_position.y;

    double min_distance = (global_feature.x-known_features[0].x)*(global_feature.x-known_features[0].x)
    					+ (global_feature.y-known_features[0].y)*(global_feature.y-known_features[0].y);
    int min_index = 0;
    for (int i=1; i<known_feature_size; i++)
    {
    	double distance = (global_feature.x-known_features[i].x)*(global_feature.x-known_features[i].x)
                        + (global_feature.y-known_features[i].y)*(global_feature.y-known_features[i].y);
        if (distance < min_distance)
        {
            min_distance = distance;
            min_index = i;
        }
    }

    if (min_distance < threshold)
    {
		p.index = min_index;
		p.distance = min_distance;
        matched_global_feature.push_back(p);
    }
}

void slam::update_angle()
{
	line temp;
	list<line>::iterator iter1 = detected_line_local.begin();
	list<line>::iterator iter2 = detected_line_local.begin();

	// Sort by length
	for ( ; iter1!=detected_line_local.end(); iter1++)
	{
		iter2 = iter1;
		for ( ; iter2!=detected_line_local.end(); iter2++)
		{
			if (iter1->length < iter2->length)
			{
				temp = *iter1;
				*iter1 = *iter2;
				*iter2 = temp;
			}
		}
	}

//	iter1 = detected_line_local.begin();
//	line lines[detected_line_local.size()];
//	int i = 0;
//	for (; iter1!=detected_line_local.end(); iter1++)
//	{
//		lines[i] = *iter1;
//		lines[i].index = i;
//		cout<< " i is "<<i <<endl;
//		cout<<"lines[i].theta"<<lines[i].theta<<endl;
//		cout<<"lines[i].length"<<lines[i].length<<endl;
//		i++;
//	}
//	cout << "Longest line " << lines[0].length << " " << lines[0].theta*180/pi << endl;

	line longest = *detected_line_local.begin();

	double tempAngle;
	double UAVangle = cur_position.theta;

	tempAngle = UAVangle + longest.theta;
	tempAngle = mod(tempAngle, pi/4);

	if (tempAngle < pi/8)
		UAVangle = UAVangle - tempAngle;
	else
		UAVangle = UAVangle - (tempAngle - pi/4);
	INPI(UAVangle);
	pre_position.theta = UAVangle;
//	cout << "Current theta is               " << UAVangle*180/pi << endl;
}

double slam::mod(double x,double y)
{
	if (x>0)
		while (x>y) x-=y;
	else
		while (x<0) x+=y;
	return x;
}
