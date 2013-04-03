#!/usr/bin/env python
import rospy
import roslib; roslib.load_manifest('base_position')
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

import numpy as np

#from gazebo_msgs.msg import LinkStates
from std_msgs.msg import String
import tf
#import subprocess

pub = rospy.Publisher('move_base_simple/goal', PoseStamped)
#rospy.init_node('base_pose')

#CONSTANTS
dist_ball_to_bridge = 0.15
table_top_height = 0.055 #HACK 
table_length = 2.4784
table_width = 1.3592
bridge_bottom_height = 0.06 + 0.2

#transform of base in bridge frame by looking up tf echo l_wrist_roll_link  base_footprint
## WTF!?!?!?!
'''
tfbb = Pose()
tfbb.position.x=1.237
tfbb.position.y=-0.66
tfbb.position.z=-0.349
tfbb.orientation.x=0.0
tfbb.orientation.y=-0.707
tfbb.orientation.z=0.0
tfbb.orientation.w=0.707
'''
#transform of base in bridge frame
tfbb = Pose()
tfbb.position.x=0.7895563460561493
tfbb.position.y=0.3611339947807817
tfbb.position.z=-0.660658868487761
tfbb.orientation.x=0.4997179635436379
tfbb.orientation.y=-0.4999773727645122
tfbb.orientation.z=-0.5000920418849375
tfbb.orientation.w=0.5002124881274455


#print 'ROLL PITCH YAW',tf.transformations.euler_from_quaternion((tfbb.orientation.x,tfbb.orientation.y,tfbb.orientation.z,tfbb.orientation.w),'sxyz')
#(tfbb.orientation.x,tfbb.orientation.y,tfbb.orientation.z,tfbb.orientation.w) = tf.transformations.quaternion_from_euler(0,0,0,'ryxz')

#transform of table in world frame
tftw = Pose()
tftw.position.x=4.0#changing
tftw.position.y=-2.0#changing
tftw.position.z=0.75
(tftw.orientation.x,tftw.orientation.y,tftw.orientation.z,tftw.orientation.w) = tf.transformations.quaternion_from_euler(0,0,np.pi/2)


#defining the hard coded position of the ball and direction of approach. We will get this from a service later
ball_x = 1.5
ball_y = 0.2
angle = np.pi/2



rospy.init_node('Base_position', anonymous=True)
tfBroadcast = tf.TransformBroadcaster()
tfListen = tf.TransformListener()


#we need subscriber code later when we will listen to  cue ball's position
def callback(data):
    bridge_position = calculate_bridge_position(ball_x, ball_y, angle)
    #b_pose=calculate_base_position(bridge_position)
    print "Yaaaay"

def listener():
    #print "someting"
    rospy.Subscriber("stupid", String, callback)
    #print "woohoo"
    rospy.spin()


def dist(x1,y1,x2,y2):
    return np.sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1))

def calculate_bridge_position(px,py,theta):
    
    ux = np.cos(theta)
    uy = np.sin(theta)
    #ux = (dx-px)/distance
    #uy = (dy-py)/distance
    bx = px - (dist_ball_to_bridge * ux)
    by = py - (dist_ball_to_bridge * uy)

    
    pose = Pose()
    pose.position.x = bx
    pose.position.y = by
    pose.position.z = table_top_height + bridge_bottom_height
    (x,y,z,w) = tf.transformations.quaternion_from_euler(0,np.pi/2,theta)#quaternion_about_axis(theta,(0,0,1)) 
    pose.orientation.x=x
    pose.orientation.y=y
    pose.orientation.z=z
    pose.orientation.w=w
    '''
    #print 'bridge position',bx,by
    pose = Pose()
    pose.position.x = 0
    pose.position.y = 0
    pose.position.z = 0
    #(x,y,z,w) = tf.transformations.quaternion_about_axis(theta,(0,0,1)) 
    pose.orientation.x=0
    pose.orientation.y=0
    pose.orientation.z=0.383
    pose.orientation.w=0.924
    #print 'bridge posE',pose
    tfBroadcast.sendTransform((pose.position.x,pose.position.y,pose.position.z),
                     (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                     rospy.Time.now(),
                     "rot_only",
                     "ORIGIN")
     
    #print 'bridge position',bx,by
    #pose = Pose()
    pose.position.x = 0.5
    pose.position.y = 0
    pose.position.z = 0
    #(x,y,z,w) = tf.transformations.quaternion_about_axis(theta,(0,0,1)) 
    pose.orientation.x=0
    pose.orientation.y=0
    pose.orientation.z=0.0
    pose.orientation.w=1
    #print 'bridge posE',pose
    tfBroadcast.sendTransform((pose.position.x,pose.position.y,pose.position.z),
                     (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                     rospy.Time.now(),
                     "tr_only",
                     "ORIGIN")

    #print 'bridge position',bx,by
    pose = Pose()
    pose.position.x = 0
    pose.position.y = 0
    pose.position.z = 0
    #(x,y,z,w) = tf.transformations.quaternion_about_axis(theta,(0,0,1)) 
    pose.orientation.x=0
    pose.orientation.y=0
    pose.orientation.z=0.383
    pose.orientation.w=0.924
    #print 'bridge posE',pose
    tfBroadcast.sendTransform((pose.position.x,pose.position.y,pose.position.z),
                     (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                     rospy.Time.now(),
                     "tr_rot",
                     "tr_only")
    '''
    #rate = rospy.Rate(10.0)
    p = pose.position
    o = pose.orientation
    #while not rospy.is_shutdown():
    print "BRIDGE WRT TABLE",(p,o)
    tfBroadcast.sendTransform((p.x, p.y, p.z),
                     (o.x, o.y, o.z, o.w),
	#	      (0,0,0,1),
                     rospy.Time.now(),
                     "bridge",
                     "table")
        #rate.sleep()
        #pose = #transformation between table and world
    p = tftw.position
    o = tftw.orientation
    print "TABLE WRT World:",(p,o)
    tfBroadcast.sendTransform((p.x, p.y, p.z),
                     (o.x, o.y, o.z, o.w),
                     rospy.Time.now(),
                     "table",
                     "/map")

    gotit=0
    br_pose = Pose()
    #while not gotit   
    try:
        #tfListen.waitForTransfrom('/bridge','/world', rospy.Time.now(),rospy.Duration(4.0))
        (trans, rot) = tfListen.lookupTransform('bridge', '/map', rospy.Time(0))
        print "HHHHHHHHHHHHHHHHHHHHHHH - BRIDGE WRT WORLD:",trans,rot
        br_pose.position.x = trans[0]
        br_pose.position.y = trans[1]
        br_pose.position.z = trans[2]
        br_pose.orientation.x = rot[0]
        br_pose.orientation.y = rot[1]
        br_pose.orientation.z = rot[2]
        br_pose.orientation.w = rot[3]
        #((br_pose.position.x, br_pose.position.y, br_pose.position.z),(br_pose.orientation.x, br_pose.orientation.y, br_orientation.z, br_pose.orientation.w)) = tfListen.lookupTransform('bridge', 'world', rospy.Time(0))
        gotit=1
        #print 'BRIDGE WRT WORLD',br_pose.position, br_pose.orientation
        #print 'BRIDGE WRT WORLD', br_pose
    except:
        print "Terrible 1"
    #return pose
    #print 'iiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiBRIDGE WRT WORLD',br_pose.position.x, br_pose.orientation.w
    

    #def calculate_base_position(br_pose):

    #we need to calculate the base position from the transform between bridge and pr2 base
    #tf_bridge_pr2base = Pose()
    if gotit==1:
        '''
        ps = br_pose.position
        os = br_pose.orientation
        tfBroadcast.sendTransform((ps.x, ps.y, ps.z),
                         (os.x, os.y, os.z, os.w),
                         rospy.Time.now(),
                         "world1",
                         "bridge1")
        '''
        pa = tfbb.position
        oa = tfbb.orientation
        tfBroadcast.sendTransform((pa.x, pa.y, pa.z),
                         (oa.x, oa.y, oa.z, oa.w),
                         rospy.Time.now(),
                         "pr2_base",
                         "bridge")
        try:
            (trans,rot) = tfListen.lookupTransform('/map','pr2_base', rospy.Time(0))
            print 'ROBOT BASE IN THEEEEEEEEE MAP',trans, rot
            new_pose = Pose()
            new_pose.position.x = trans[0]
            new_pose.position.y = trans[1]
            new_pose.position.z = trans[2]
            new_pose.orientation.x = rot[0]
            new_pose.orientation.y = rot[1]
            new_pose.orientation.z = rot[2]
            new_pose.orientation.w = rot[3]
            talker(new_pose)
            #print "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa", new_pose
        except:
            print "Terrible 2"


def talker(base_pose):
    #while not rospy.is_shutdown():
    #str = "hello world %s" % rospy.get_time()
    #    rospy.loginfo(str)
    stamped = PoseStamped()
    stamped.pose=base_pose
    stamped.header.frame_id = '/map'
    stamped.header.stamp = rospy.Time.now()
    #print "HURRRRRRRRAY",stamped
    pub.publish(stamped)
    #    rospy.sleep(1.0)

if __name__ == '__main__':
#    bridge_position = calculate_bridge_position(ball_x, ball_y, angle)
#    calculate_base_position(bridge_position)
     listener()
