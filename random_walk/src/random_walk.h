/**
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
 *  File: <random_walk.h>
 *  Desc: <Header file for random_walk.cpp>
 *  Auth: <Dylan John Drover>
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

#ifndef RANDOM_WALK_H
#define RANDOM_WALK_H

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "random_walk/random_walkConfig.h"
#include "nav_msgs/Odometry.h"

//Used to determine the heading of the robot (direction_choice())
enum TURN_STATE {
	RIGHT         = 0,
	LEFT          = 1,
	FORWARD       = 2,
	BLOCKED_LEFT  = 3,
	BLOCKED_RIGHT = 4,
};

class RandomWalk {

	public:

		//Constructor for random_walk class
		RandomWalk();

		//Updates the parameters of random_walk from dynamic reconfigure
		void update_cb(random_walk::random_walkConfig&, uint32_t);

		//Called after a random duration of time and sets random direction for robot
		void timer_cb(const ros::TimerEvent&);

		//Processes the input laserscan and determines if there is an obstacle to avoid
		void laser_cb(const sensor_msgs::LaserScan::ConstPtr&);
		
		//Sets the linear and angular velocities of the robot based off current TURN_STATE
		void direction_choice();
		
		//Will determine if the robot is stuck in the environment and turn it around until it
		//breaks free
		void trapped_release();

		//Updates the yaw member, used to track how far the robot has turned (used in simulation)
		void odom_cb(const nav_msgs::Odometry&);

		//Updates the yaw member, used to track how far the robot has turned (used in real world)
		void ekf_cb(const geometry_msgs::PoseWithCovarianceStamped&);

		//Swath of angles the laser scanner will have processed
		int view_angle;
		//Threshold distance that tells the robot to avoid object
		double min_dist;
		//Width of robot used to determine its forward rectangle
		double robot_width;
		//Set linear speed, used to set linear_vel
		double linear_max;
		//Set angular speed, used to set angular_vel
		double angular_max;
		//Current (updates) linear velocity
		double linear_vel;
		//Current (updates) angular velocity
		double angular_vel;
		//Frame of the laser taken in from dynamic reconfigure, used to transform the laserscan range
		std::string sensor_frame_id;
		//The rotation of the laser scanner extracted from sensor_frame_id
		double trans_angle;
		//Current rotation of robot (z-axis) set by odom or robot_ekf
		double yaw; 
		//When the robot is stuck, this is used as a reference as the first yaw value
		double first_yaw;
		//How far the robot turns around 
		double turn_around;
		//Published limited scan, only shows the data within the view angles. 
		//Useful for showing in rviz to see the scope of what the robot is processing.
		sensor_msgs::LaserScan lim_scan;
		//Bool flags used for trapped_release()
		bool blocked;
		bool stuck_r;
		bool stuck_l;
		//Current heading of the robot
		TURN_STATE direction;
		//One iteration previous heading of robot 
		TURN_STATE prev_state;

};

#endif
