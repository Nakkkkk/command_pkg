#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#include <tf/transform_datatypes.h>

#include "std_msgs/Int16.h"

class rsj_robot_test_node
{
private:
	ros::Subscriber sub_odom;
	ros::Subscriber sub_scan;
	ros::Subscriber sub_key;
	ros::Publisher pub_twist;

	sensor_msgs::LaserScan latest_scan;
	geometry_msgs::Twist cmd_vel;
	std_msgs::Int16 current_command;

	void cb_odom(const nav_msgs::Odometry::ConstPtr &msg)
	{
	}
	void cb_scan(const sensor_msgs::LaserScan::ConstPtr &msg)
	{
		latest_scan = *msg;
	}
	void cd_key(const std_msgs::Int16::ConstPtr& msg)
	{
		current_command = *msg;
//		ROS_INFO("I heard in cd_key: [%d]", current_command.data);
		if(current_command.data == 1)
		{
			ROS_INFO("cd_key goto_points");
			mainloop();
		}
		else if(current_command.data == 2)
		{
			ROS_INFO("cd_key shutdown");

			cmd_vel.linear.x = 0.0;
			cmd_vel.angular.z = 0.0;
			pub_twist.publish(cmd_vel);

			ros::shutdown();
		}
	}
public:
    rsj_robot_test_node()
    {
        ros::NodeHandle nh("~");
        pub_twist = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
        sub_odom = nh.subscribe("/odom", 5, &rsj_robot_test_node::cb_odom, this);
        sub_scan = nh.subscribe("/scan", 5, &rsj_robot_test_node::cb_scan, this);
	sub_key = nh.subscribe("/command", 1000, &rsj_robot_test_node::cd_key, this);

	ros::spin();
    }
    void mainloop()
    {
        ROS_INFO("Hello ROS World!");

//	geometry_msgs::Twist cmd_vel;

        ros::Rate rate(10.0);

        while(ros::ok())
        {
            ros::spinOnce();

			if(latest_scan.ranges.size() > 0)
			{
//				geometry_msgs::Twist cmd_vel;
				int size = 2*(-latest_scan.angle_min) / latest_scan.angle_increment;
				int i = size/2;
				int j = 3*size/8;
				int k = 5*size/8;
				if(latest_scan.ranges[i] < latest_scan.range_min ||
						latest_scan.ranges[i] > latest_scan.range_max || 
						std::isnan(latest_scan.ranges[i]))
				{
					ROS_INFO("front-range: measurement error");
					cmd_vel.linear.x = 0.1;
					cmd_vel.angular.z = 0.0;
				}
				else
				{
					ROS_INFO("front-range: %0.3f", latest_scan.ranges[i]);
					if(latest_scan.ranges[i] > 0.5)
					{
						cmd_vel.linear.x = 0.1;
						cmd_vel.angular.z = 0.0;
					}
					else if(latest_scan.ranges[k] < 0.5)
					{
						cmd_vel.linear.x = 0.0;
						cmd_vel.angular.z = -0.8;
					}
					else if(latest_scan.ranges[j] < 0.5)
					{
						cmd_vel.linear.x = 0.0;
						cmd_vel.angular.z = 0.8;
					}
					else if(latest_scan.ranges[i] < 0.5)
					{
						cmd_vel.linear.x = 0.0;
						cmd_vel.angular.z = 1.2;
					}
				}
				pub_twist.publish(cmd_vel);
			}

            rate.sleep();
        }
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rsj_robot_test_node");

    rsj_robot_test_node robot_test;

    robot_test.mainloop();
}
