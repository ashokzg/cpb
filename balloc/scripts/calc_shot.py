#! /usr/bin/env python

import roslib; roslib.load_manifest('balloc')
import rospy

# Brings in the SimpleActionClient
import actionlib

import math
from billiards_msgs.msg import *

def bill_planner_client(tbl_state):
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('plan_shot', PlanShotAction)

    # Waits until the action server has started up and started
    # listening for goals.
    if client.wait_for_server() == False:
        print "Server not available"
    else:
        print "Found server"

    # Creates a goal to send to the action server.
    goal = billiards_msgs.msg.PlanShotActionGoal()
    
    goal.goal.state = tbl_state
    goal.goal.angle_max = math.pi
    goal.goal.angle_min = -math.pi

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result(300)
    x = billiards_msgs.msg.PlanShotActionResult()
    # Prints out the result of executing the action
    shotResult = client.get_result()  
    print shotResult

def tblCb(data):
    print "Got table state"
    bill_planner_client(data)

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('calc_shot')
        rospy.Subscriber("table_state", TableState, tblCb)        
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
    rospy.spin()