#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include "std_msgs/String.h"

using namespace std;

class rsj_robot_test_node
{
private:
        ros::Publisher pub_goal;
	tf::TransformListener tfl;

        ros::Subscriber sub_points;
        ros::Subscriber sub_key;

	std::list<geometry_msgs::PoseStamped> goals;
	geometry_msgs::PoseStamped current_goal;
	geometry_msgs::PointStamped current_point;
	std_msgs::String current_command;

	void cd_point(const geometry_msgs::PointStamped& msg)
	{
		ROS_INFO("cd_point called");
		current_point = msg;
	}
	void cd_key(const std_msgs::String::ConstPtr& msg)
	{
		ROS_INFO("I heard in cd_key: [%s]", msg->data.c_str());
		current_command.data = msg->data;
	}

public:

    rsj_robot_test_node()
    {
        ros::NodeHandle nh;
	sub_points = nh.subscribe("/clicked_point", 1000, &rsj_robot_test_node::cd_point, this);
	sub_key = nh.subscribe("/command", 1000, &rsj_robot_test_node::cd_key, this);
        pub_goal = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 5, true);
	ROS_INFO("A");

    }

	void add_goal(const geometry_msgs::PointStamped& msg)
	{
		if(msg.point.x != 0 && msg.point.y != 0)
		{
//		ROS_INFO("C");
		geometry_msgs::PoseStamped goal;
		goal.header.frame_id = "map";
		goal.pose.position.x = msg.point.x;
		goal.pose.position.y = msg.point.y;
		goal.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
		goals.push_back(goal);
		ROS_INFO("push_back");
		}
//		ROS_INFO("B");
	}

	bool pop_goal()
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

	void stamp_points()
	{
		ROS_INFO("B");
		ros::Rate rate(5.0);
		std::string s = "s";

		while(ros::ok)
		{
			ROS_INFO("plz stamp points where you want to move.");
			ROS_INFO("plz pless S button to move");
//			ROS_INFO(current_command);

			while(current_command.data != std::string("s"))
			{
				ROS_INFO("I heard in stamp_points: [%s]", current_command.data.c_str());

				rate.sleep();
				add_goal(current_point);
			}
			
			ROS_INFO("get s cmd");

			mainloop();
		}
	}

    void mainloop()
	{
		ROS_INFO("Hello ROS World!");
		ros::Rate rate(5.0);

//	while(ros::ok())
//	{
//		rate.sleep();
//		ros::spinOnce();

//		add_goal(current_point);

		if(!pop_goal())
		{
//			ROS_ERROR("waitting destination points");
			ROS_ERROR("No goal specified");
//			return;
//			break;
		}else{

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
				if(!pop_goal())
				{
					ROS_INFO("Finished");
					//return;
					break;
				}
				ROS_INFO("Next goal applied");
			}
		}
		}
	}
//	}
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rsj_robot_test_node");

    rsj_robot_test_node robot_test;

	// 行き先を追加
//	robot_test.add_goal(0.3, -0.3, 0.0);
//	robot_test.add_goal(0.2, 0.2, 1.57);

//    robot_test.mainloop();
    robot_test.stamp_points();
}
