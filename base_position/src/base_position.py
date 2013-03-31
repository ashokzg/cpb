#!/usr/bin/env python
import rospy
import roslib; roslib.load_manifest('base_position')
from geometry_msgs.msg import Pose
import numpy as np

#from gazebo_msgs.msg import LinkStates
from std_msgs.msg import String
import tf
#import subprocess

#CONSTANTS
dist_ball_to_bridge = 0.15
table_top_height = 0.805 
table_length = 2.4784
table_width = 1.3592
bridge_bottom_height = 0.06
#transform of base in bridge frame
#Tfbb.pos.x=0
#Tfbb.pos.y=0
#Tfbb.pos.z=0
#Tfbb.ori.x=0
#Tfbb.ori.y=0
#Tfbb.ori.z=0
#Tfbb.ori.w=0
#transform of table in world frame
tftw = Pose()
tftw.position.x=-2.0
tftw.position.y=-4.0
tftw.position.z=0.75
(tftw.orientation.x,tftw.orientation.y,tftw.orientation.z,tftw.orientation.w) = tf.transformations.quaternion_from_euler(0,0,0,'ryxz')


#defining the hard coded position of the ball and direction of approach. We will get this from a service later
ball_x = 1.000
ball_y = 1.000
angle =  np.pi/4



rospy.init_node('Base_position', anonymous=True)
tfBroadcast = tf.TransformBroadcaster()
tfListen = tf.TransformListener()


#we need subscriber code later when we will listen to  cue ball's position
def callback(data):
    bridge_position = calculate_bridge_position(ball_x, ball_y, angle)
    #calculate_base_position(bridge_position)


def listener():
    print "someting"
    rospy.Subscriber("stupid", String, callback)
    print "woohoo"
    rospy.spin()


def dist(x1,y1,x2,y2):
    return np.sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1))

def calculate_bridge_position(px,py,theta):


    '''
    #calculating the x and y for bridge base position
    m = np.tan(theta)
    print 'm',m
    #with segment x=0
    dx=0
    dy=py - m
    side1= dist(px,py,dx,dy)
    c=dy

    #with segment x = (table_length)
    dx = table_length
    dy = m*table_length - c
    side2 = dist(px,py,dx,dy)

    #with segment y=0
    dy=0
    dx=-c/m
    side3 = dist(px,py,dx,dy)

    #with segment y=table_width
    dy=table_width
    dx=(dy-c)/m 
    side4 = dist(px,py,dx,dy)
 
    side=np.argmin([side1,side2,side3,side4])
    if side==0:
        dx=0
        dy=py - m
        distance = dist(px,py,dx,dy)
        

    if side==1:
        dx = table_length
        dy = m*table_length - c
        distance = dist(px,py,dx,dy)

    if side==2:
        dy=0
        dx=-c/m
        distance = dist(px,py,dx,dy)

    if side==3:
        dy=table_width
        dx=(dy-c)/m 
        distance = dist(px,py,dx,dy)

    print 'intersection point',dx,dy
    '''
    ux = np.sin(theta)
    uy = np.cos(theta)
    #ux = (dx-px)/distance
    #uy = (dy-py)/distance
    bx = px + (dist_ball_to_bridge * ux)
    by = py + (dist_ball_to_bridge * uy)

    print 'bridge position',bx,by

    pose = Pose()
    pose.position.x = bx
    pose.position.y = by
    pose.position.z = table_top_height + bridge_bottom_height
    (x,y,z,w) = tf.transformations.quaternion_about_axis(theta,(0,0,1))  #the bridge will be perpendicular to the shot angle
    pose.orientation.x=x
    pose.orientation.y=y
    pose.orientation.z=z
    pose.orientation.w=w
    print 'bridge posE',pose
#    tfBroadcast.sendTransform((pose.position.x, pose.position.y, pose.position.z),
#                     (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
#                     rospy.Time.now(),
#                     "bridge",
#                     "table")
     

    rate = rospy.Rate(10.0)
    #while not rospy.is_shutdown():
    tfBroadcast.sendTransform((9,8,7),
                     (9,8,7,6),
                     rospy.Time.now(),
                     "bridge",
                     "table")
        #rate.sleep()
        #pose = #transformation between table and world
    p = tftw.position
    o = tftw.orientation
    tfBroadcast.sendTransform((p.x, p.y, p.z),
                     (o.x, o.y, o.z, o.w),
                     rospy.Time.now(),
                     "table",
                     "world")

   
    try:
        tfListen.waitForTransfrom('/bridge','/world', rospy.Time.now(),rospy.Duration(4.0))
        (pose.position,pose.orientation) = tfListen.lookupTransform('/bridge', '/world', rospy.Time(0))
        print 'BRIDGE WRT WORLD',pose.position, pose.orientation
    except:
        print "Terrible"
    return pose
    

def calculate_base_position(br_pose):

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
    except:
        print "Terrible"


if __name__ == '__main__':
#    bridge_position = calculate_bridge_position(ball_x, ball_y, angle)
#    calculate_base_position(bridge_position)
     listener()
