#!/usr/bin/env python
import rospy
import roslib; roslib.load_manifest('ik_trajectory_tutorial')

import tf
from ik_trajectory_tutorial.srv import *
from geometry_msgs.msg import *
import gazebo
from gazebo_msgs.msg import *


rospy.init_node('adjust_left_arm', anonymous=True)
br2world = tf.TransformBroadcaster()
br2pr2 = tf.TransformListener()


def adjust_arm(req):
    print req.arm, req.adjust, req.globalpose
    x = AdjustArmResponse(5)
    print "Resp:", x
    return x
    curTr = rospy.wait_for_message("gazebo/link_states", gazebo_msgs.msg.LinkStates , 5)
    
    br2world.sendTransform(req.globalpose.position, 
                           req.globalpose.orientation, 
                           rospy.Time.now(),
                           "pr2_left_gripper_des",
                           "absolute_origin")

if __name__ == "__main__":
    s = rospy.Service('adjust_arm', AdjustArm, adjust_arm)
    rospy.spin()
