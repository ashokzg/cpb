#!/usr/bin/env python
import roslib; roslib.load_manifest('simple_trajectory')

import sys

import rospy
from simple_trajectory.srv import *


def move_joint_client(side, joint, angle, vel):
    print "Waiting for service"
    rospy.wait_for_service('move_one_joint')
    print "Found service"
    try:
        move_joint = rospy.ServiceProxy('move_one_joint', MoveJoint)
        success = move_joint(side, joint, angle, vel)
        return success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s side[0 to 1] jointIdx[0 to 6] angle velocity"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 5:
        print sys.argv[3]
        side = int(sys.argv[1])%2
        joint = int(sys.argv[2])%7
        angle = float(sys.argv[3])
        vel = float(sys.argv[4])
    else:
        print usage()
        sys.exit(1)
    print side, joint, angle, vel
    ret = move_joint_client(side, joint, angle, vel)
    print "Current Joint configuration:\n----------------------"
    for j in ret.curPosition:
        print j
