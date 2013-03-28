#!/usr/bin/env python
import rospy
import roslib; roslib.load_manifest('bridge_mover')

from simple_trajectory.srv import *
from simple_gripper.srv import *
from ik_trajectory_tutorial.srv import *
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
import tf

init_right = [-1.8425060510635376,1.2198034524917603,-2.2416832447052,-1.7119375467300415,1.6620752811431885,-0.41495949029922485,-1.3718557357788086]  
init_left  = [1.3493558168411255,-0.30538955330848694,1.2200419902801514,-1.244804859161377,1.3725829124450684,-1.989667534828186,-3.133694648742676]

def move_joint_client(side, ang):
    print "Waiting for service"
    rospy.wait_for_service('move_multiple_joints')
    print "Found service"
    try:
        move_joint = rospy.ServiceProxy('move_multiple_joints', MoveMultipleJoints)
        success = move_joint(side, ang)
        return success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

stick_des_trans = [-0.0912116604503, -0.351264326918-0.01, 0.395577986182]
stick_des_quat  = [0, 0, 0, 1]

def setStick():
    print "Waiting for Gazebo link state"
    rospy.wait_for_service('gazebo/set_model_state')
    print "Setting stick"
    link = ModelState()
    link.model_name = "stick"
    link.reference_frame = ""
    link.pose.position.x = stick_des_trans[0]
    link.pose.position.y = stick_des_trans[1]
    link.pose.position.z = stick_des_trans[2]
    link.pose.orientation.x = stick_des_quat[0]
    link.pose.orientation.y = stick_des_quat[1]
    link.pose.orientation.z = stick_des_quat[2]
    link.pose.orientation.w = stick_des_quat[3]
    link.twist.linear.x = 0.0
    link.twist.linear.y = 0.0
    link.twist.linear.z = 0.0
    link.twist.angular.x = 0.0
    link.twist.angular.y = 0.0
    link.twist.angular.z = 0.0    
    try:
        setStickClient = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)
        print "Client is ", setStickClient
        resp = setStickClient(link)
        print "Success state:", resp.success
        print "Status message:", resp.status_message
        return resp.success
    except:
        print "Setting the stick state in gazebo failed"

def changeJoints(side, oc):
    print "Waiting for gripper service"
    rospy.wait_for_service('gazebo/set_link_state')
    print "changing gripper"   
    try:
        grip = rospy.ServiceProxy('gripper', ChangeGripper)
        ret = grip(side, oc)
        print "Success status: ", ret.success
    except:
        print "Gripping failed"

if __name__ == "__main__":
    ret = True
    ret = move_joint_client(0, init_right)
    if ret == False:
        print "ERROR: Initializing right arm failed"
    ret = move_joint_client(1, init_left)
    if ret == False:
        print "ERROR: Initializing right arm failed"  
    changeJoints(0,0)
    changeJoints(1,0)    
    rospy.sleep(15)    
    setStick()
    changeJoints(0,1)
    changeJoints(1,1)    
    rospy.sleep(15)      
    print "Current Joint configuration:\n----------------------"
