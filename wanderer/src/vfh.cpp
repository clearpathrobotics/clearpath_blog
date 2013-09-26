/*
 *      _____
 *     /  _  \
 *    / _/ \  \
 *   / / \_/   \
 *  /  \_/  _   \  ___  _    ___   ___   ____   ____   ___   _____  _   _
 *  \  / \_/ \  / /  _\| |  | __| / _ \ | ++ \ | ++ \ / _ \ |_   _|| | | |
 *   \ \_/ \_/ /  | |  | |  | ++ | |_| || ++ / | ++_/| |_| |  | |  | +-+ |
 *    \  \_/  /   | |_ | |_ | ++ |  _  || |\ \ | |   |  _  |  | |  | +-+ |
 *     \_____/    \___/|___||___||_| |_||_| \_\|_|   |_| |_|  |_|  |_| |_|
 *             ROBOTICS™ 
 *
 *  File: vfh.cpp
 *  Desc: Vector Field Histogram method for navigation 
 *  Auth: Dylan John Drover
 *
 *  Copyright (c) 2013, Clearpath Robotics, Inc. 
 *  All Rights Reserved
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * Please send comments, questions, or patches to skynet@clearpathrobotics.com 
 *
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_listener.h"

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#include "vfh.h"

//Determines the unknown length of a side of a triangle
inline double cos_law(double a, double b, double theta)
{
	return sqrt(pow(a,2) + pow(b,2) - 2*a*b*cos(theta)); 
}

VFH::VFH()
{
	alert_threshold = 0.75;
	turn_threshold = 6;
	robot_width = 0.65;
	linear_max = 0.5;
	angular_max = 0.8;
	ave_count = 0;
    trans_angle = 0;
    turning = false;
    dead_end = false;
    
}

void VFH::display_heading_marker(double heading_point, double distance, int marker_id)
{	
	visualization_msgs::Marker marker;

	marker.header.frame_id = "base_link";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = marker_id;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.25;
	marker.scale.y = 0.25;
	marker.scale.z = 0.25;
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.25;

	marker.pose.position.x = distance * sin(heading_point);
	marker.pose.position.y = distance * -cos(heading_point);
	marker.pose.position.z = 0.3;

	m_array.markers.push_back(marker);
	marker_id++;

}

//Updates variables through dynamic reconfigure
void VFH::update_cb(wanderer::wanderConfig &config, uint32_t level)
{
	ROS_DEBUG("Reconfigure Request: %f %f %f %f %f %s", config.alert_threshold, config.turn_threshold, config.robot_width, config.linear_max, config.angular_max, config.sensor_frame_id.c_str());
	
    alert_threshold = config.alert_threshold;
    turn_threshold  = config.turn_threshold;
	robot_width     = config.robot_width;
	linear_max      = config.linear_max;
	angular_max     = config.angular_max;
    sensor_frame_id = config.sensor_frame_id;
}

void VFH::find_min_max(std::vector<double> &input_array)
{
	for(int i = 0; i < input_array.size(); i++)
	{
        if(input_array[i] < min_distance) {
            min_distance = input_array[i];
            danger_zone = i * increment;
        } else if(input_array[i] > max_distance) {
        	max_distance = input_array[i];
        }
        //Used to keep track of the percentage of points under threshold
        if(input_array[i] <= alert_threshold) {
        	percent++;
       	}
    }
}

void VFH::passable_depth(std::vector<POI> &POI_array, std::vector<double> &input_array, std::vector<POI> &output)
{
	for(int i = 0; i < (POI_array.size() - 1); i++) {
		int index_a = POI_array[i].angle/increment;
		int index_b = POI_array[i+1].angle/increment;
		double dist_ave = 0;
		for(int j = index_a; j < index_b; j++) {
			dist_ave += input_array[j];
		}
		dist_ave = dist_ave/(index_b - index_a);

		//ROS_INFO("Gap Average: %f, Simple Ave: % f", dist_ave, ((input_array[index_a] + input_array[index_b])/2));

		if(dist_ave > ((input_array[index_a] + input_array[index_b])/2)+1) { 
			output.push_back(POI_array[i]);
			output.push_back(POI_array[i+1]);
			//ROS_INFO("Gap Average: %f, Simple Ave: % f", dist_ave, ((input_array[index_a] + input_array[index_b])/2));
		}
	}
}

void VFH::passable_width(std::vector<POI> &list)
{
	for (int i = 0; i < list.size(); i+=2) {
		double passable = cos_law(list[i].distance, list[i+1].distance, fabs(list[i].angle - list[i+1].angle));
		
		if ((passable > robot_width)) { //&& (within_thresh)
			double x = ((list[i].distance * cos(list[i].angle)) + (list[i+1].distance * cos(list[i+1].angle)))/2;
			double y = ((list[i].distance * sin(list[i].angle)) + (list[i+1].distance * sin(list[i+1].angle)))/2;

			double dir_angle = atan2(y, x);
			//ROS_INFO("GAP WIDTH: %f", passable);
		    if(sqrt(pow(x,2) + pow(y,2)) > alert_threshold) {
		        gaps.push_back(dir_angle);
		    }
		}
	}
}

void VFH::edge_detector(std::vector<double> &input_array)
{
	std::vector<POI> all_edges;
	std::vector<double> zeroed_array;
	 
	//Zeros the out of bounds members
	for(int i = 0; i < input_array.size(); i++) {
		if(input_array[i] > turn_threshold) {
		        zeroed_array.push_back(0);
		} else {
			//within_threshold set here
			zeroed_array.push_back(input_array[i]);   
		}
	}
	
	//Finds abrupt chnage in the data and puts it in the first tier of edges
    for (int i = 1; i < (zeroed_array.size() - 1); i++) {    
	    POI temp;
	    if ((zeroed_array[i+1] - zeroed_array[i]) < -0.5) {
	        temp.distance = zeroed_array[i];
	        temp.angle = i * increment;
	        temp.type = 0;

	        all_edges.push_back(temp);
	    } else if (zeroed_array[i+1] - zeroed_array[i] > 0.5) {
	        temp.distance = input_array[i+1];
	        temp.angle = (i+1) * increment;
	        temp.type = 1;

	        all_edges.push_back(temp);
	    }
	}
	
	//Used for trouble shooting, not necessary for regular running
	/*
    gnuplot_cmd(hdl, "plot '-' using 1:2 \n");
    for(int i = 0; i < zeroed_array.size(); i++)
    {
        double x = i * increment;
        double y = zeroed_array[i];
        gnuplot_cmd(hdl,"%f %f \n", x, y);
    }
    gnuplot_cmd(hdl, "e \n");
    */
	
	//Checks to see if area behind edge pair is suitable
	if(all_edges.size() > 1) {
		passable_depth(all_edges, input_array, edge_list);
	}

	//Removes redundant edges from edge_list 
	if (edge_list.size() > 0) {
		std::vector<POI>::iterator i = edge_list.begin();
		while (i != (edge_list.end()-1)) {
		    if ((i+1)->type == i->type) {
		        if (i->type == 0) {
		             edge_list.erase(i+1);    
		        } else if (i->type == 1) {
		            i = edge_list.erase(i);
		        }
		    } else { 
		        i++;
		    }
		}
	}

	//Displays first tier of edges 
	if(edge_list.size() > 0) {
		for (int k = 0; k < edge_list.size(); k++) {
			//ROS_INFO("Angle: %f, Distance: %f, Type: %d", edge_list[k].angle, edge_list[k].distance, edge_list[k].type);
		}
		//Places the proper heading into gaps if viable
		passable_width(edge_list);
	}
	
	all_edges.clear();
	zeroed_array.clear();
}

void VFH::corner_detector(std::vector<double> &input_array)
{
	
	//Gradient threshold is 70 degrees
	double gradient_threshold = 1.22; 
	
	//Sets up x and y coordinates for gradient as well as creates
	//vector of all gradients between all the points in the laserscan
	for (int i = 0; i < (input_array.size() - 1); i++) {
		double x0 = input_array[i] * cos(i*increment);
		double y0 = input_array[i] * sin(i*increment);
		double x1 = input_array[i+1] * cos((i+1)*increment);
		double y1 = input_array[i+1] * sin((i+1)*increment);

		Vector temp;
		temp.x = x1 - x0;
		temp.y = y1 - y0;
		temp.i = i;
		temp.distance = input_array[i];
		temp.slope = fabs(atan2(temp.y,temp.x));
		gradients.push_back(temp);
	}

	//Temporary arrays to store sets of corners 
	std::vector<POI> first_tier;
	std::vector<POI> second_tier;
	double grad_ave_a = 0;
	double grad_ave_b = 0;
	int k = 10;
	
	//Find the average gradient over span of 20 points for each point in Laserscan
	while (k < (gradients.size() - 10)) {
		for (int j = -10; j < 10; j++) { 
		    if(j < 0)
		        grad_ave_a +=gradients[k+j].slope;
		    else
		        grad_ave_b +=gradients[k+j].slope;
		}
		grad_ave_a = grad_ave_a/10;
		grad_ave_b = grad_ave_b/10;
		
		if((fabs(grad_ave_a - grad_ave_b) > gradient_threshold) && (gradients[k].distance < turn_threshold)) { 
		    POI temp;
		    temp.angle = gradients[k].i*increment;
		    temp.distance = gradients[k].distance;
		    temp.type = 1;
		    
		    first_tier.push_back(temp);
		}
		k++;
	}
	
	//There must be 2 corners to proceed. Will determine is the area between the corners 
	//is clear behind it
	if(first_tier.size() >= 2) {
		passable_depth(first_tier, input_array, second_tier);
	}

	//Will add the way points of the corners to the viable paths
	if(second_tier.size() >= 2) { 
		passable_width(second_tier);
	}
	
	gradients.clear();
	first_tier.clear();
	second_tier.clear();
}

//Potnetial solution, currently not used though the code does work.
double VFH::potential_field(std::vector<double> &input_array) 
{
    double positive_field = 0;
    double negative_field = 0;
    double potential_fields = 0;
    int count_p = 0;
    int count_n = 0;
    for(int i = 0; i < input_array.size(); i++) {
        if(input_array[i] > alert_threshold) {
            positive_field += i*increment;
            count_p++;
        } else {
            negative_field += (i*increment - M_PI);
            count_n++;
        }
    }
    if(count_n > 0) {
        negative_field = negative_field/count_n;
        positive_field = positive_field/count_p;
        double x = (count_p*cos(positive_field)) + (count_n*cos(negative_field));
        double y = (count_p*sin(positive_field)) + (count_n*sin(negative_field));

        potential_fields = atan2(y, x);
        
        return potential_fields;
    } else {
        return NULL;
    }
}

void VFH::smoother(const sensor_msgs::LaserScan::ConstPtr& scan, int range)
{
	if (ave_count == 0) {
        for (int i = 0; i < range; i++) {
            moving_ave_array.push_back(scan->ranges[i]);
		}
        ave_count++;
    } else if (ave_count < 5) {
        for (int i = 0; i < range; i++) {
            moving_ave_array[i] += scan->ranges[i];
		}
        ave_count++;
    } else if(ave_count == 5) {
    	for (int i = 0; i < range; i++) {
            moving_ave_array[i] = moving_ave_array[i]/5;
		}
		ave_count++; //Casues exit condition 
	}
}

void VFH::laser_cb(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	min_distance = 100;
    danger_zone = 0;
    max_distance = 0;
	increment = scan->angle_increment;
    double min_scan_angle = scan->angle_min + trans_angle;
    double max_scan_angle = scan->angle_max;
    int laser_scan_range;
    int count = 0;
    double chosen_heading;
    percent = 0;
    
    //Sets up the range depending of the laser scanner (~180 or ~360)
    if((fabs(max_scan_angle - min_scan_angle) <= (M_PI+0.1)) || (fabs(max_scan_angle - min_scan_angle) >= (M_PI-0.1))) {
        laser_scan_range = scan->ranges.size();
    } else if((fabs(max_scan_angle - min_scan_angle) <= (2*M_PI+0.1)) || (fabs(max_scan_angle - min_scan_angle) >= (2*M_PI-0.1))) {
        laser_scan_range = scan->ranges.size()/2;
    }
	

	//Smooths the incoming data
    if(ave_count <= 5) {
    	smoother(scan, laser_scan_range);
    } else {
    	find_min_max(moving_ave_array);
    	
    	//Find if there are possible paths
    	double top = percent;
		double bottom = laser_scan_range;
		//Tries to find edges, if none are found, it tries to find corners, if not...see below
		edge_detector(moving_ave_array);
		//ROS_INFO("Min: %f, Max: %f, Angle: %f", min_distance, max_distance, danger_zone);
    	if(gaps.size() == 0) {
			corner_detector(moving_ave_array);
		}
		
	 	if (gaps.size() > 0) { 
	 		//turning = false;
	 		set_velocities(chosen_heading);
	 		current = FINE;
			ROS_INFO("Lin: %f Ang: %f", linear_vel, angular_vel);
		} else {
			linear_vel  = linear_max;
			angular_vel = 0;
			//turning = false;
		}
		
		if(((fabs(cos(danger_zone))*min_distance) < (robot_width/2)) && (min_distance < alert_threshold)) {
			if (cos(danger_zone) > 0) {
				angular_vel = angular_max;
				current = BLOCKED_RIGHT;
				ROS_INFO("BLOCKED RIGHT");
			} else {
				angular_vel = -(angular_max);
				current = BLOCKED_LEFT;
				ROS_INFO("BLOCKED LEFT:");
			}
			linear_vel = 0;
		}
		
		//if(((current == BLOCKED_RIGHT) && (previous == BLOCKED_LEFT)) || ((current == BLOCKED_LEFT) && (previous == BLOCKED_RIGHT))) {
			//stuck_condition();
		//}
		
    	edge_list.clear();
    	gaps.clear();
    	moving_ave_array.clear();
    	ave_count = 0;
    } 
}

void VFH::stuck_condition()
{
	if(!turning) {
		first_yaw = yaw;
		turning = true;
	} else if(fabs(yaw - first_yaw) < (M_PI/2)) {
		linear_vel = 0;
		angular_vel = -(angular_max);
	} else {
		turning = false;
	}
	ROS_INFO("Difference %f, %f", yaw, first_yaw);
	
}

void VFH::set_velocities(double &heading)
{
	double max_mag = 0;
	double selection_function;
	
	double chosen_heading;

	//Take out as separate function  
	if ((gaps.size() > 0)) {
		for (int i = 0; i < gaps.size(); i++) {
			ROS_INFO("Gap jazz,: %f", gaps[i]);
			if (gaps[i] <= (M_PI/2)) {
				selection_function = ((2/M_PI)*gaps[i]) + (1/max_distance)*moving_ave_array[gaps[i]/increment];
			} else if (gaps[i] > (M_PI/2)) {
				selection_function = (2 - (2/M_PI)*gaps[i]) + (1/max_distance)*moving_ave_array[gaps[i]/increment];
			}
			if (selection_function > max_mag) {
				max_mag = selection_function;
				chosen_heading = gaps[i];
				visualization_msgs::Marker marker;
				display_heading_marker(chosen_heading, moving_ave_array[gaps[i]/increment], i);
			}
		}
	}

	//Take out as separate function (set the heading velocities)
   	//Linear scaling of angular and linear velocities
	angular_vel = -(angular_max) + ((2/M_PI)*chosen_heading*angular_max);
	if ((chosen_heading <= (M_PI/2)) && (chosen_heading > 0.35)) {
		linear_vel = ((2/M_PI)*chosen_heading*linear_max);
	} else if ((chosen_heading > (M_PI/2)) && (chosen_heading < (M_PI - 0.35))) {
		linear_vel = 2*linear_max - ((2/M_PI)*chosen_heading*linear_max);
	 //else if (min_distance < alert_threshold) {
		//linear_vel = 0;
	} else {
		linear_vel = linear_max;
		angular_vel = 0;
	}
}

//Can track the yaw of the robot updating member "yaw" in simulation
void VFH::odometry_cb(const nav_msgs::Odometry& odom)
{
    double roll;
    double pitch;
    btQuaternion quat(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    btMatrix3x3(quat).getEulerZYX(yaw, pitch, roll);
}

//Can track the yaw of the robot updating member "yaw" in real world
void VFH::ekf_cb(const geometry_msgs::PoseWithCovarianceStamped& odom)
{
    double roll;
    double pitch;
    btQuaternion quat(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    btMatrix3x3(quat).getEulerZYX(yaw, pitch, roll);
}
