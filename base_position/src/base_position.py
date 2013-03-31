#!/usr/bin/env python
import rospy
import roslib; roslib.load_manifest('base_position')
from geometry_msgs.msg import Pose
import numpy as np

dist_ball_to_bridge = 0.15
table_top_height = 1
#defining the hard coded position of the ball and direction of approach. We will get this from a service later
ball_x = 1.0
ball_y = 0.25
angle = np.pi/2


#from gazebo_msgs.msg import LinkStates
from std_msgs.msg import String
import tf
#import subprocess

#rospy.init_node('fingerfinder', anonymous=True)
tfBroadcast = tf.TransformBroadcaster()
tfListen = tf.TransformListener()

''' we need subscriber code later when we will listen to  cue ball's position
def callback(data):


def listener():
    print "someting"
    rospy.Subscriber("/gazebo/link_states", LinkStates, callback)
    print "woohoo"
    rospy.spin()
'''
def calculate_bridge_position(px,py,theta):

    pz = table_top_height
    pose = Pose()
    pose.position.x = px
    pose.position.y = py
    pose.position.z = pz
    (x,y,z,w) = tf.transformations.quaternion_from_euler((theta-np.pi/2),0,0)  #the bridge will be perpendicular to the shot angle
    pose.orientation.x=x
    pose.orientation.y=y
    pose.orientation.z=z
    pose.orientation.w=w
    tfBroadcast.sendTransform([pose.position.x, pose.position.y, pose.position.z],
                     [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
                     rospy.Time.now(),
                     "bridge_base",
                     "table")
    #pose = #transformation between bridge and bridge base
    p = pose.position
    o = pose.orientation
    tfBroadcast.sendTransform([p.x, p.y, p.z],
                     [o.x, o.y, o.z, o.w],
                     rospy.Time.now(),
                     "bridge",
                     "bridge_base")
    #pose = #transformation between table and world
    p = pose.position
    o = pose.orientation
    tfBroadcast.sendTransform([p.x, p.y, p.z],
                     [o.x, o.y, o.z, o.w],
                     rospy.Time.now(),
                     "table",
                     "world")
    try:
        (pose.position,pose.orientation) = tfListen.lookupTransform('/bridge', '/world', rospy.Time(0))
        print pose.position, pose.orientation
    except:
        print "Terrible"
    return pose
    

def calculate_base_position(br_pose)

    #we need to calculate the base position from the transform between bridge and pr2 base
    tf_bridge_pr2base = Pose()

    p = br_pose.position
    o = br_pose.orientation
    tfBroadcast.sendTransform([p.x, p.y, p.z],
                     [o.x, o.y, o.z, o.w],
                     rospy.Time.now(),
                     "bridge",
                     "world")
    p = tf_bridge_pr2base.position
    o = tf_bridge_pr2base.orientation
    tfBroadcast.sendTransform([p.x, p.y, p.z],
                     [o.x, o.y, o.z, o.w],
                     rospy.Time.now(),
                     "pr2_base",
                     "bridge")
    try:
        (trans,rot) = tfListen.lookupTransform('/pr2_base', '/world', rospy.Time(0))
        print trans, rot

if __name__ == '__main__':
    bridge_position = calculate_bridge_position(ball_x, ball_y, angle)
    calculate_base_position(bridge_position)

