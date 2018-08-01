#!/usr/bin/env python
# license removed for brevity
import rospy, sys
import readchar
from std_msgs.msg import Int16

def key_command():

    pub = rospy.Publisher('/command', Int16, queue_size=10)
    rospy.init_node('key_command')
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():

      kb = readchar.readchar()
      sys.stdout.write(kb)

      if kb == 's':
        print("get s key")
        pub.publish(1)
      elif kb == 'q':
        print("get q key")
        pub.publish(2)
        break

      r.sleep()

if __name__ == '__main__':
    try:
        key_command()
    except rospy.ROSInterruptException: pass
