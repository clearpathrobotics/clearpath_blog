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
 *  File: rnd_pub.cpp
 *  Desc: Random Walk publisher for twist controlled robots
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

#include <math.h>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "dynamic_reconfigure/server.h"
#include <tf/transform_listener.h>

#include "random_walk.h"

RandomWalk::RandomWalk()
{
	trans_angle = 0;
	//Number of radians the robot will turn to find new edge
	turn_around = (M_PI/4);
	view_angle = 90;
	blocked = 0;
	stuck_r = 0;
	stuck_l = 0;
	linear_max = 0.3;
	angular_max = 0.3;
}

void RandomWalk::update_cb(random_walk::random_walkConfig &config, uint32_t level)
{
	ROS_DEBUG("Reconfigure Request: %f %d %f %f %f %s",config.min_dist, config.view_angle, config.robot_width, config.linear_vel, config.angular_vel, config.sensor_frame_id.c_str());
	
	min_dist        = config.min_dist;
	view_angle      = config.view_angle;
	robot_width     = config.robot_width;
	linear_max      = config.linear_vel;
	angular_max     = config.angular_vel;
	sensor_frame_id = config.sensor_frame_id;
}

//Sends either left or right command at random
void RandomWalk::timer_cb(const ros::TimerEvent&)
{
	//if TURN_STATE is 1, robot turns left, right on zero, if 2 it goes straight 
	if(!blocked)
		direction = (TURN_STATE) (rand() % 3);
}

void RandomWalk::laser_cb(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	double dist = 1000;
	int indx = 0;
	lim_scan.header.stamp = ros::Time::now();

	//Identify the area in front of the robot and then determine 
    //the important (max and min) values in said area
	double min_ang = (-view_angle) * (M_PI/180);
	double max_ang = (view_angle) * (M_PI/180);
	double increm = scan->angle_increment;
	int size = scan->ranges.size();

    //Start & end of laserscan ranges values for viewing cone
	int start = size/2 + min_ang/increm - trans_angle/increm;
	int end = size/2 + max_ang/increm - trans_angle/increm;
	
	//Copy relevant data to broadcasted limited scan
	lim_scan.scan_time = scan->scan_time;
	lim_scan.angle_increment = increm;
	lim_scan.time_increment = scan->time_increment;
	lim_scan.range_min = scan->range_min;
	lim_scan.range_max = scan->range_max;
		
	lim_scan.ranges.resize(end - start);
	
	double angle;
	int count = 0;
	//Determines the minimum distance in desired frame
	for (int i = start; i < end; i++) {
		lim_scan.ranges[count] = scan->ranges[i]; 
        if (lim_scan.ranges[count] < dist) {
			dist = lim_scan.ranges[count];
			indx = i;
			angle = indx*increm;
		}
		count++;
	}
	//ROS_INFO("Angle: %f Dist: %f Cos: %f", indx*increm*(180/M_PI), dist, (fabs(cos(angle))*dist));

	//Throws exception and turns away from close object (closer than min_dist) by using 
    //trigonometry to determine if an object is within the forward rectangle of the robot.
    //If the cosine of the angle is > 0, then the object is on the right side (this is if the laser 
    //sweeps from right to left, index starts on the right side.
    if (((fabs(cos(angle))*dist) < (robot_width/2)) && (dist < min_dist)) {
		if (cos(angle) > 0) 
			direction = BLOCKED_RIGHT;
		else 
			direction = BLOCKED_LEFT;
		
		//Blocked flag to prevent random turn
		blocked = 1;
		dist = 1000;
		indx = 0;
	} else {
		//Allows random turns 
		blocked = 0;
		dist = 1000;
		indx = 0; 
	}

}

void RandomWalk::direction_choice()
{
	switch(direction) {
		case BLOCKED_LEFT:
			linear_vel = 0;
			angular_vel = -(angular_max);
			ROS_DEBUG("Blocked on Left");
			break;
		case BLOCKED_RIGHT:
			linear_vel = 0;
			angular_vel = angular_max;
			ROS_DEBUG("Blocked on Right");
			break;
		case LEFT:
			linear_vel = angular_max;
			angular_vel = linear_max;
			ROS_DEBUG("LEFT");
			break;
		case RIGHT:
		 	linear_vel = linear_max;
			angular_vel = -(angular_max);
			ROS_DEBUG("RIGHT");
			break;
		case FORWARD:
			linear_vel = linear_max;
			angular_vel = 0;
			ROS_DEBUG("FORWARD");
			break;
		default:
			linear_vel = 0;
			angular_vel = 0;
			break;
    }
}

//If the robot is blocked in a corner this detects said case and will
//turn for "turn_around" radians. If the robot caught an edge, it will
//continue on. If not it will continue to turn until is finds its way out
void RandomWalk::trapped_release()
{
	if(((prev_state == BLOCKED_RIGHT) && (direction == BLOCKED_LEFT)) || (stuck_r)) {
		if(!stuck_r)
			first_yaw = yaw;

		stuck_r = 1;
		if(fabs(first_yaw - yaw) <= turn_around) 
			direction = BLOCKED_RIGHT;
		else 
			stuck_r = 0;

		} else if(((prev_state == BLOCKED_LEFT) && (direction == BLOCKED_RIGHT)) || (stuck_l)) {
		if(!stuck_l)
			first_yaw = yaw;

		stuck_l = 1;
		if(fabs(first_yaw - yaw) <= turn_around)
			direction = BLOCKED_LEFT;
		else 
			stuck_l = 0;
	}
}

//Used if in simulation
void RandomWalk::odom_cb(const nav_msgs::Odometry& odom)
{
    double roll;
    double pitch;
    btQuaternion quat(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    btMatrix3x3(quat).getEulerZYX(yaw, pitch, roll);
}

//Used on real robot
void RandomWalk::ekf_cb(const geometry_msgs::PoseWithCovarianceStamped& odom)
{
    double roll;
    double pitch;
    btQuaternion quat(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    btMatrix3x3(quat).getEulerZYX(yaw, pitch, roll);
}

