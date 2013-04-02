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

#req = AdjustArmRequest()
def adjust_arm(req):
    print req.arm, req.adjust, req.globalpose
    x = AdjustArmResponse(5)
    print "Resp:", x
    #return x
    curTr = rospy.wait_for_message("gazebo/link_states", gazebo_msgs.msg.LinkStates , 5)
    p = req.globalpose.position
    o = req.globalpose.orientation
    o.w = 1.0
    br2world.sendTransform((p.x, p.y, p.z), 
                           (o.x, o.y, o.z, o.w), 
                           rospy.Time.now(),
                           "pr2_left_gripper_des",
                           "absolute_origin")
    i = curTr.name.index('pr2::base_footprint')                       
    p = curTr.pose[i].position
    o = curTr.pose[i].orientation

    br2world.sendTransform((p.x, p.y, p.z), 
                           (o.x, o.y, o.z, o.w), 
                           rospy.Time.now(),
                           "pr2_base",
                           "absolute_origin")

    i = curTr.name.index('pr2::l_gripper_motor_slider_link') 
    p = curTr.pose[i].position
    o = curTr.pose[i].orientation

    br2world.sendTransform((p.x, p.y, p.z), 
                           (o.x, o.y, o.z, o.w), 
                           rospy.Time.now(),
                           "pr2_left_gripper_cur",
                           "absolute_origin")
    (trans,rot) = br2pr2.lookupTransform('pr2_left_gripper_cur', 'pr2_base', rospy.Time(0))

if __name__ == "__main__":
    s = rospy.Service('adjust_arm', AdjustArm, adjust_arm)
    rospy.spin()
