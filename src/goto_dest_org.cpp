#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

class rsj_robot_test_node
{
private:
	ros::Publisher pub_goal;
	tf::TransformListener tfl;

	std::list<geometry_msgs::PoseStamped> goals;
	geometry_msgs::PoseStamped current_goal;

public:
	rsj_robot_test_node()
	{
	        ros::NodeHandle nh("~");
		pub_goal = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 5, true);
	}
	void add_goal(const float x, const float y, const float yaw)
	{
		geometry_msgs::PoseStamped goal;
		goal.header.frame_id = "map";
		goal.pose.position.x = x;
		goal.pose.position.y = y;
		goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
		goals.push_back(goal);
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
	void mainloop()
	{
		ROS_INFO("Hello ROS World!");

		if(!pop_goal())
		{
			ROS_ERROR("No goal specified");
			return;
		}

		ros::Rate rate(10.0);
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
					return;
				}
				ROS_INFO("Next goal applied");
			}
		}
	}
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "rsj_robot_test_node");

	rsj_robot_test_node robot_test;

	robot_test.add_goal(0.3, -0.3, 0.0);

	robot_test.mainloop();
}

