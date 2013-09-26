#include <math.h>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>

#include "ros/ros.h"
#include "dynamic_reconfigure/server.h"
#include <tf/transform_listener.h>

#include "random_walk.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "random_walk");
	ros::NodeHandle nh;
	geometry_msgs::Twist dir;
	RandomWalk random_walk;

	//The viewing angle is passed to the ladser_cb as an extra parameter
	ros::Publisher laser_pub = nh.advertise<sensor_msgs::LaserScan>("lim_scan", 100);
	ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("base_scan/scan", 100, &RandomWalk::laser_cb, &random_walk);
	ros::Publisher rand_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    
    ros::Subscriber ekf_sub;
    ros::Subscriber odom_sub;
    bool real_flag = 0; 
    nh.getParamCached("/random_walk/real_flag", real_flag);

    if(real_flag) {
        ekf_sub = nh.subscribe("robot_pose_ekf/odom", 100, &RandomWalk::ekf_cb, &random_walk);
    } else {
        odom_sub = nh.subscribe("odom", 100, &RandomWalk::odom_cb, &random_walk);
	}
	
	ros::Timer loop = nh.createTimer(ros::Duration(5 + rand() % 10), &RandomWalk::timer_cb, &random_walk);
	ros::Rate rate(20); // 20hz

	dynamic_reconfigure::Server<random_walk::random_walkConfig> server;
	dynamic_reconfigure::Server<random_walk::random_walkConfig>::CallbackType update_fields;

	update_fields = boost::bind(&RandomWalk::update_cb, &random_walk, _1, _2);
	server.setCallback(update_fields);
	
	tf::StampedTransform rel_frame;
	tf::TransformListener listen;
	tf::Quaternion rotation;
    
    //Waits for tf from laser scan	
	try {
		ros::Time now = ros::Time::now();
		listen.waitForTransform(random_walk.sensor_frame_id, "/base_link", now, ros::Duration(5.0));
		listen.lookupTransform(random_walk.sensor_frame_id, "/base_link", ros::Time(0), rel_frame);
	    rotation = rel_frame.getRotation();
	    random_walk.trans_angle = rotation.getAngle();
	} catch (tf::TransformException ex) {
		 ROS_ERROR("%s",ex.what());
	}

	while (ros::ok()) {
		rate.sleep();
        
        //Checks for trapped condition
        random_walk.trapped_release();
        //Sets the velocities of the differential drive robot
        random_walk.direction_choice();
        
        dir.linear.x = random_walk.linear_vel;
        dir.angular.z = random_walk.angular_vel;
        rand_pub.publish(dir);
      	
        random_walk.prev_state = random_walk.direction; 
            
		random_walk.lim_scan.header.frame_id = random_walk.sensor_frame_id;

		laser_pub.publish(random_walk.lim_scan);
		ros::spinOnce();
	}
	
	return 0;
}

