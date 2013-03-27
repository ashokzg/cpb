#!/usr/bin/env python
import rospy
import roslib; roslib.load_manifest('bridge_mover')

from gazebo_msgs.msg import LinkStates
from std_msgs.msg import String
import tf
import subprocess

rospy.init_node('fingerfinder', anonymous=True)
br = tf.TransformBroadcaster()
br2pr2 = tf.TransformListener()
def callback(data):
    #rospy.loginfo("hello")
    #print data.name[46]
    #print data.name
    left_gripper_l = data.name.index('pr2::l_gripper_l_finger_tip_link')
    left_gripper_r = data.name.index('pr2::r_gripper_l_finger_tip_link')
    right_gripper_l = data.name.index('pr2::r_gripper_l_finger_tip_link')
    right_gripper_r = data.name.index('pr2::r_gripper_r_finger_tip_link')
    bridge = data.name.index('stick::bridge')
    fp = data.name.index('pr2::base_footprint')
    lever = data.name.index('stick::lever_stick')
    print "Bridge", data.pose[bridge]
    print "Foot print", data.pose[fp]
    
    
    rx = (data.pose[right_gripper_l].position.x + data.pose[right_gripper_r].position.x)/2.
    ry = (data.pose[right_gripper_l].position.y + data.pose[right_gripper_r].position.y)/2.
    rz = (data.pose[right_gripper_l].position.z + data.pose[right_gripper_r].position.z)/2.
    lx = (data.pose[left_gripper_l].position.x + data.pose[left_gripper_r].position.x)/2.
    ly = (data.pose[left_gripper_l].position.y + data.pose[left_gripper_r].position.y)/2.
    lz = (data.pose[left_gripper_l].position.z + data.pose[left_gripper_r].position.z)/2.
    print "Right x, y, z: ", rx, ry, rz
    print "Left x, y, z: ", lx, ly, lz    
    print "Bride wrt gplane", data.pose[bridge]
    print "Lever wrt gplane", data.pose[lever]    
    p = data.pose[bridge].position
    o = data.pose[bridge].orientation
    br.sendTransform([p.x, p.y, p.z],
                     [o.x, o.y, o.z, o.w],
                     rospy.Time.now(),
                     "bridge",
                     "gplane")
    p = data.pose[fp].position
    o = data.pose[fp].orientation
    br.sendTransform([p.x, p.y, p.z],
                     [o.x, o.y, o.z, o.w],
                     rospy.Time.now(),
                     "foot_print",
                     "gplane")
    try:
        (trans,rot) = br2pr2.lookupTransform('/bridge', '/foot_print', rospy.Time(0))
        print trans, rot
    except:
        print "Terrible"
    #rospy.loginfo(rospy.get_name() + ": I heard %s" % data.data)

def listener():
    print "someting"
    rospy.Subscriber("/gazebo/link_states", LinkStates, callback)
    print "woohoo"    
    rospy.spin()

if __name__ == '__main__':
    listener()
