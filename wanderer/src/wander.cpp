#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_listener.h"
#include "dynamic_reconfigure/server.h"

#include "vfh.h"

int main(int argc, char **argv)
{  
	ros::init(argc, argv, "wander");
	ros::NodeHandle node;
	VFH vfh;
    
	ros::Subscriber laser = node.subscribe("base_scan/scan", 100, &VFH::laser_cb, &vfh);
    ros::Publisher way_point = node.advertise<visualization_msgs::MarkerArray>("cmd_pnt", 1);
    ros::Publisher dir = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	ros::Rate rate(20);
	
	dynamic_reconfigure::Server<wanderer::wanderConfig> server;
	dynamic_reconfigure::Server<wanderer::wanderConfig>::CallbackType update_fields;
	
	update_fields = boost::bind(&VFH::update_cb, &vfh, _1, _2);
	server.setCallback(update_fields);
    
    ros::Subscriber ekf_sub;
    ros::Subscriber odometry_sub;
    bool real_flag = 1; 
    node.getParamCached("/wander/real_flag", real_flag);
    
    //vfh.hdl = gnuplot_init();
     
    if (real_flag) {
        ekf_sub = node.subscribe("robot_pose_ekf/odom", 100, &VFH::ekf_cb, &vfh);
    } else {
        odometry_sub = node.subscribe("odom", 100, &VFH::odometry_cb, &vfh);
    }
    
    tf::StampedTransform rel_frame;
	tf::TransformListener listen;
    tf::Quaternion rotation;
    
    //Waits for tf from laser scan	
	try {
		ros::Time now = ros::Time::now();
		listen.waitForTransform(vfh.sensor_frame_id, "/base_link", now, ros::Duration(5.0));
		listen.lookupTransform(vfh.sensor_frame_id, "/base_link", ros::Time(0), rel_frame);
	    rotation = rel_frame.getRotation();
	    vfh.trans_angle = rotation.getAngle();
	 } catch (tf::TransformException ex) {
		 ROS_ERROR("%s",ex.what());
	 }

	while (ros::ok()) {
        geometry_msgs::Twist heading;
		
	    heading.linear.x = vfh.linear_vel;
	    heading.angular.z = vfh.angular_vel;
	    dir.publish(heading);

		way_point.publish(vfh.m_array);
		vfh.m_array.markers.clear();
		
		//vfh.previous = vfh.current;

		rate.sleep();
		ros::spinOnce();
	}
    gnuplot_close(vfh.hdl);	
	return 0;
}
