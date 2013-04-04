#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import roslib; roslib.load_manifest('bridge_mover')

from gazebo_msgs.msg import LinkStates
from gazebo_msgs.msg import GetModelState




def callback(data):
    rospy.loginfo(rospy.get_name() + ": I heard %s" % data.data)


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", String, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
#this is allakai, ignore!!!
