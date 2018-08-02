#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include "std_msgs/Int16.h"

using namespace std;

class robot_node
{
private:
        ros::Subscriber sub_points;
        ros::Subscriber sub_key;
        ros::Publisher pub_goal;

	tf::TransformListener tfl;

	std::list<geometry_msgs::PoseStamped> goals;
	geometry_msgs::PoseStamped current_goal;
	geometry_msgs::PointStamped current_point;
	std_msgs::Int16 current_command;

	void cd_point(const geometry_msgs::PointStamped& msg)
	{
		current_point = msg;
//		ROS_INFO("I heard in cd_point: [%f]", current_point.point.x);
		stack_points();
	}
	void cd_key(const std_msgs::Int16::ConstPtr& msg)
	{
		current_command = *msg;
//		ROS_INFO("I heard in cd_key: [%d]", current_command.data);
		if(current_command.data == 1)
		{
			ROS_INFO("cd_key goto_points");
			goto_point();
		}
		else if(current_command.data == 2)
		{
			ROS_INFO("cd_key shutdown");
			ros::shutdown();
		}
	}

public:
	robot_node()
	{
		ros::NodeHandle nh;

		sub_points = nh.subscribe("/clicked_point", 1000, &robot_node::cd_point, this);
		sub_key = nh.subscribe("/command", 1000, &robot_node::cd_key, this);

		pub_goal = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 5, true);

//		repeat_stack();

		ROS_INFO("A");

		ros::spin();
	}

	void repeat_stack()
	{
		ROS_INFO("repeat at repeat");
		while(ros::ok)
		{
			stack_points();
/*
			if(!pop_points())
			{
				ROS_ERROR("No goal specified");
			}
			else
			{
				goto_point();
			}
*/
			if(current_command.data = 1)
			{
				goto_point();
			}
			else if(current_command.data = 2)
			{
				ros::shutdown();
			}
		}
	}
	void stack_points()
	{
//		while(current_command.data != 1)
//		{
			ROS_INFO("I heard in stack_point: [%f]", current_point.point.x);

			if(current_point.point.x != 0.0 && current_point.point.y != 0.0)
			{
				ROS_INFO("C");
				geometry_msgs::PoseStamped goal;
				goal.header.frame_id = "map";
				goal.pose.position.x = current_point.point.x;
				goal.pose.position.y = current_point.point.y;
				goal.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
				goals.push_back(goal);
				ROS_INFO("push_back");
			}
			else
			{
				ROS_INFO("B");
			}
//		}
	}
	bool pop_points()
	{
		if(goals.size() == 0) return false;

		current_goal = goals.front();
		goals.pop_front();
		ROS_INFO("Applying goal %0.3f %0.3f %0.3f",
				current_goal.pose.position.x,
				current_goal.pose.position.y,
				tf::getYaw(current_goal.pose.orientation));
		pub_goal.publish(current_goal);

		return true;
	}
	void goto_point()
	{
		ROS_INFO("Hello ROS World!");
		ros::Rate rate(5.0);

		if(!pop_points())
		{
			ROS_ERROR("No goal specified");
			return;
		}
		while(ros::ok())
		{
			rate.sleep();
			ros::spinOnce();

			float x, y, yaw;
			try
			{
				tf::StampedTransform trans;
				tfl.waitForTransform("map", "base_link", 
						ros::Time(0), ros::Duration(0.5));
				tfl.lookupTransform("map", "base_link", 
						ros::Time(0), trans);
				x = trans.getOrigin().x();
				y = trans.getOrigin().y();
				yaw = tf::getYaw(trans.getRotation());
			}
			catch(tf::TransformException &e)
			{
				ROS_WARN("%s", e.what());
				continue;
			}

			float yaw_goal = tf::getYaw(current_goal.pose.orientation);
			float yaw_error = yaw - yaw_goal;
			if(yaw > M_PI) yaw -= 2.0 * M_PI;
			else if(yaw < -M_PI) yaw += 2.0 * M_PI;

			if(hypotf(x - current_goal.pose.position.x,
						y - current_goal.pose.position.y) < 0.15 &&
					fabs(yaw_error) < 0.3)
			{
				if(!pop_points())
				{
					ROS_INFO("Finished");
					return;
				}
				ROS_INFO("Next goal applied");
			}
		}
	}

};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "robot_node");

	robot_node robot;
	ROS_INFO("repeat at main");
//	robot.repeat_stack();

}
