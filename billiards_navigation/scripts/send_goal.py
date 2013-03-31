#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib
roslib.load_manifest('billiards_navigation')

import actionlib
import rospy
import sys
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction

USAGE = 'USAGE: send_goal frame x y z qx qy qz qw'

ACTION_NAME = 'drive_pool'

def main(frame, x, y, z, qx, qy, qz, qw):
    rospy.init_node('send_goal')

    client = actionlib.SimpleActionClient(ACTION_NAME, MoveBaseAction)
    print 'Waiting for %s action...'%(ACTION_NAME)
    client.wait_for_server()

    g = MoveBaseGoal()
    g.target_pose.header.frame_id = frame
    g.target_pose.pose.position.x = x
    g.target_pose.pose.position.y = y
    g.target_pose.pose.position.z = z
    g.target_pose.pose.orientation.x = qx
    g.target_pose.pose.orientation.y = qy
    g.target_pose.pose.orientation.z = qz
    g.target_pose.pose.orientation.w = qw

    print g

    print 'Sending goal: ' + `g`
    client.send_goal(g)
    print 'Waiting for action to complete...'
    client.wait_for_result()

if __name__ == '__main__':
    args = rospy.myargv()

    if len(args) != 9:
        print USAGE
        sys.exit(1)

    main(args[1], float(args[2]), float(args[3]), float(args[4]), float(args[5]), float(args[6]), float(args[7]), float(args[8]))

