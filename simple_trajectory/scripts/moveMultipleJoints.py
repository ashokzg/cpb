#!/usr/bin/env python
import roslib; roslib.load_manifest('simple_trajectory')

import sys

import rospy
from simple_trajectory.srv import *


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

def usage():
    return "%s side[0 to 1] jointAngle1 jointAngle2 ... jointAngle7"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 9:
        side = int(sys.argv[1])
        ang = [0]*7
        for i in range(7):
            ang[i] = float(sys.argv[i+2])
    else:
        print usage()
        sys.exit(1)
    ret = move_joint_client(side, ang)
    #print "Current Joint configuration:\n----------------------"

