#!/usr/bin/env python
import roslib; roslib.load_manifest('bridge_mover')

import sys

import rospy

from gazebo_msgs.msg import LinkStates
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import *

from std_msgs.msg import String

req=GetModelStateRequest()

def ball_pose_client(model_name,relative_entity_name):
    rospy.wait_for_service('gazebo/get_model_state')
    try:
        print "try"
        ball_pose = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)

#        req=GetModelStateRequest(ball_pose)
#        req.model_name='ball1'
#        req.relative_entity_name=''

#        print ball_pose
        resp1 = ball_pose(model_name, relative_entity_name)
#        resp2 = GetModelStateRequest("relative_entity_name",'')


#        print resp1
#        print resp2
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

#def usage():
#    return "%s [model_name] [relative_entity_name]"%sys.argv[0]

if __name__ == "__main__":

#    ball_pose_client()
    model_name = 'table_model'
    relative_entity_name = ''
    res = ball_pose_client(model_name,relative_entity_name)
    print "Ball Pose"
    print res



    

