#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#include <tf/transform_datatypes.h>

#include "std_msgs/Int16.h"

class robot_node
{
private:
	ros::Subscriber sub_odom;
	ros::Subscriber sub_scan;
	ros::Publisher pub_twist;
        ros::Subscriber sub_key;

	sensor_msgs::LaserScan latest_scan;
	std_msgs::Int16 current_command;

	void cb_scan(const sensor_msgs::LaserScan::ConstPtr &msg)
	{
		latest_scan = *msg;
		get_sensor();
	}
	void cd_key(const std_msgs::Int16::ConstPtr& msg)
	{
		current_command = *msg;
//		ROS_INFO("I heard in cd_key: [%d]", current_command.data);

		if(current_command.data == 2)
		{
			ROS_INFO("cd_key shutdown");
			ros::shutdown();
		}
	}


public:
	robot_node()
	{
		ros::NodeHandle nh("~");
		pub_twist = nh.advertise<geometry_msgs::Twist>("/my_robo/diff_drive_controller/cmd_vel", 5);
		sub_scan = nh.subscribe("/scan", 5, &robot_node::cb_scan, this);
		sub_key = nh.subscribe("/command", 1000, &robot_node::cd_key, this);
	}

	void get_sensor()
	{
		ros::Rate rate(10.0);
		int i = (-latest_scan.angle_min) / latest_scan.angle_increment;

		while(ros::ok())
		{
			ROS_INFO("get sensor [%f]", latest_scan.ranges[i]);
			rate.sleep();
		}

	}

};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "robot_node");

	robot_node robot;

//	robot.get_sensor();
}
