#!/usr/bin/env python

import roslib; roslib.load_manifest('billiards_planner')
import rospy

import optparse
import sys
import time

import actionlib
from billiards_msgs.msg import BallState, PlanShotAction, PlanShotGoal, TableState

def update_table_state(table):
    table_width = 1.12
    
    # Cue ball
    cue = BallState()
    cue.id = 0
    cue.point.point.x = 0.5
    cue.point.point.y = table_width / 2
    table.balls.append(cue)

    # 1 ball
    ball_1 = BallState()
    ball_1.id = 1
    ball_1.point.point.x = 1.0
    ball_1.point.point.y = table_width / 2
    table.balls.append(ball_1)

    # 2 ball
    ball_2 = BallState()
    ball_2.id = 1
    ball_2.point.point.x = 1.0
    ball_2.point.point.y = table_width / 2 + 0.3
    table.balls.append(ball_2)

if __name__ == '__main__':
    rospy.init_node('test_plan_shot')

    table_state_sub = rospy.Subscriber('rviz_table_state', TableState, table_state_cb)

    client = actionlib.SimpleActionClient('plan_shot', PlanShotAction)
    client.wait_for_server()

    goal = PlanShotGoal()

    update_table_state(goal.state)

    client.send_goal(goal)
    client.wait_for_result()
    print client.get_result()
