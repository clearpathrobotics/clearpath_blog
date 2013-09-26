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
 *  File: vfh.h
 *  Desc: Header file for vfh.cpp
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

#ifndef VFH_H
#define VFH_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/Odometry.h"
#include "wanderer/wanderConfig.h"

#include <stdlib.h>
#include <stdio.h>
#include <string>

extern "C" {
    #include "gnuplot_i.h"
}

//Point of interest struct for corners and edges 
typedef struct {
    double distance;
    double angle;
    bool type; // 0 - first, 1 - end
} POI;

//Struct used to contain information for gradient detween points 
typedef struct {
    double i;
    double distance; 
    double x;
    double y;
    double slope;
} Vector;

enum TURN_STATE {
	BLOCKED_LEFT  = 0,
	BLOCKED_RIGHT = 1,
	FINE 		  = 2,
};

//Vector Field Histogram Class
class VFH {
	public:
		//Limit at which the robot may turn away from object 
		double alert_threshold;
		//Beyond this number the laserscan is attenuated to zero
		double turn_threshold;
		//Width of the robot used for object avoidance
		double robot_width;
		//Maximum linear velocity of the robot
		double linear_max;
		//Maximum angular velocity of the robot
		double angular_max;
		//The difference in radians between two points in the laserscan
		double increment;
		//The current set linear velocity of the robot
		double linear_vel;
		//The current set angular velocity of the robot
		double angular_vel;
        //Frame of the laser scanner used to transform the frame
        std::string sensor_frame_id;
        //Rotation of the above frame
        double trans_angle;
        //Flag of the robot 
        bool dead_end;
        //
        bool turning;
        //The max distance of the current laserscan
        double max_distance;
        //The min distance of the current laserscan
        double min_distance;
        //The angle at which the minimum distance is at
        double danger_zone;
        //
        bool within_thresh;
		//First angle of the robot when the robot becomes stuck
		double first_yaw;
		//Current angle of the robot, updated from odom or robot_ekf
		double yaw;
		//Counter for the smoother
		int ave_count;
		//The number of points that are within the alert_threshold
		int percent;
		
		//Pose of the robot
		tf::Quaternion pose;
		//Array for the possible gaps the robot can go through 
		std::vector<double> gaps;
		//Moving average array
		std::vector<double> moving_ave_array;
		//Array of points to be shown in Rviz of gaps
		visualization_msgs::MarkerArray m_array;
		
		//List of edges, first tier of edges to be filtered
		std::vector<POI> edge_list;
		//List of gradients between points in the laserscan
		std::vector<Vector> gradients;
		
		//gnuplot controller (used for debugging)
		gnuplot_ctrl *hdl;
		
		//Used for detecting a stuck condition
		TURN_STATE current;
		TURN_STATE previous;
	
		//Constructor for the VFH object
		VFH();
		
		//Takes in a angle, distacne and id to add to a marker array that will be published
		//and shown by rviz
		void display_heading_marker(double, double, int);
		
		//Updates variables through dynamic reconfigure
		void update_cb(wanderer::wanderConfig&, uint32_t);
		
		//Loops through the laserscan array and sets the maximum and minimum
		void find_min_max(std::vector<double>&);
		
		//Checks to see if the average distance of all points between two points of interest
		//is larger than the average distance of the two points. If it is, then the gap is added to
		//the output vector to continue on
		void passable_depth(std::vector<POI>&, std::vector<double>&, std::vector<POI>&);
		
		//Determines whether the gap is wide enough using the cosine law. If the gap is large
		//enough, it will be added to the final list of viable headings for the robot.
		void passable_width(std::vector<POI>&);
		
		//Takes in the laserscan array information. It then attenuates the data outside of the 
		//turn_threshold to zero such that it can limit the scope as well as more easier detect edges.
		//Detects edges by noticing a drop off of 50cm from the previous data point. 
		//denotes these as "first" and "end" points and will use this to determine gaps
		//in between the edges. Eliminates redundant edges based off of type. Will 
		//determine if gaps are passable and viable then add to gap vector.
		void edge_detector(std::vector<double>&);
		
		//Takes gradients between all points in laser scan then detects corners by comparing the average 
		//gradient over 20 data points comparing the first and last ten to see if the change in angle is 
		//greater than the gradient_threshold this is then added to the first_tier. Then iff the distance 
		//between the two corners is far away these are added to the second_tier and then if the gap is
		//wide enough the heading is added to the gaps vector 
		void corner_detector(std::vector<double>&);
		
		//
		double potential_field(std::vector<double>&);
		
		//Simple average smoother that takes in a direct laserscan type as well as the range of the scan
		//that will be averaged. Uses the ave_count within the class to keep track of number
		//of iterations, exits after 5 data sets.
		void smoother(const sensor_msgs::LaserScan::ConstPtr&, int);
		
		//Callback initialized by the robot's laserscanner. Calls the edge and corner detectors as well as 
		//handles setting the velocities of the robot.
		void laser_cb(const sensor_msgs::LaserScan::ConstPtr&);
		
		//If robot is within conditions where it appears to be stuck in a corner or dead end, this function 
		//will have the robot turn around a set number of degrees (tracked by odom or robot_ekf) such that the
		//robot can find its way once again
		void stuck_condition();
		
		//Uses the gaps member to find the most viable path (based off proximity to centre as well as distance)
		//The various headings are displayed in rviz and the best choice is made and the heading is set. The 
		//linear and angular velocities are set based on the angle of the heading
		void set_velocities(double&);

		//Updates the yaw member, used to track how far the robot has turned (used in simulation)
		void odometry_cb(const nav_msgs::Odometry&);
		
		//Updates the yaw member, used to track how far the robot has turned (used in real world)
		void ekf_cb(const geometry_msgs::PoseWithCovarianceStamped&);

};

#endif
