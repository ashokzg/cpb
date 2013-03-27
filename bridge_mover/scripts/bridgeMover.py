#!/usr/bin/env python
import rospy
import roslib; roslib.load_manifest('bridge_mover')

from gazebo_msgs.msg import LinkState
from std_msgs.msg import String

#Translation = [0.947, 0.427, 2.385]
Translation = [0.871, -0.209, 2.150]
Rotation = [0.7071067811865476, 0.7071067811865475, 0, 0]


def bridge():
    pub = rospy.Publisher('gazebo/set_link_state', LinkState)
    rospy.init_node('bridge_mover')
    while not rospy.is_shutdown():
#        link = gazebo_msgs.msg.LinkState()
        link = LinkState()
        link.link_name = "pr2::bridge"
        link.reference_frame = "pr2::base_footprint"
        link.pose.position.x = Translation[0]
        link.pose.position.y = Translation[1]
        link.pose.position.z = Translation[2]
        link.pose.orientation.x = Rotation[0]
        link.pose.orientation.y = Rotation[1]
        link.pose.orientation.z = Rotation[2]
        link.pose.orientation.w = Rotation[3]
        link.twist.linear.x = 0.0
        link.twist.linear.y = 0.0
        link.twist.linear.z = 0.0
        link.twist.angular.x = 0.0
        link.twist.angular.y = 0.0
        link.twist.angular.z = 0.0
        pub.publish(link)
        rospy.sleep(0.0005)

if __name__ == '__main__':
    try:
        bridge()
    except rospy.ROSInterruptException:
        pass
