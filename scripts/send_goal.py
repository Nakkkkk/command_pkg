#!/usr/bin/python

import rospy, actionlib
from move_base_msgs.msg import *

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String

if __name__ == '__main__':
  try:
    rospy.init_node('send_goal',anonymous=True)
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
#    sub = rospy.Subscriber('')
    pub = rospy.Publisher('goal_sign', String, queue_size=1)

    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.pose.position.x = -1.35790061951;
    goal.target_pose.pose.position.y = -1.8763718605;
    goal.target_pose.pose.orientation.w = 1
    print goal
    client.send_goal(goal)
#    print client.wait_for_result()

    success = client.wait_for_result(rospy.Duration(60))
    
    if not success:
      client.cancel_goal()
      rospy.loginfo("could not find goal in 60 sec.")
    else:
      state = client.get_state()
#      if state == GoalStatus.SUCCEEDED:
      if state == 3:
        rospy.loginfo("yattaze.")
        pub.publish("goal")
      elif state == 0:
        rospy.loginfo("aaaaaaaa")
        pub.publish("stay")
      else:
        rospy.loginfo("bbbbbbbb")
        pub.publish("else")

  except rospy.ROSInterruptException: pass
