#! /usr/bin/env python

import roslib
roslib.load_manifest('billiards_executive')

import rospy
import actionlib
from smach import *

from actionlib_msgs.msg import *
from billiards_msgs.msg import *
from geometry_msgs.msg import *

class PlayFixed(object):
    def __init__(self):
        self._shot_plan = rospy.Publisher('shot_plan', ShotPlan)
    
    def print_result(self, userdata, status, result):
        print "RESULT:", result
    
    def publish_plan_shot_result(self, userdata, status, result):
        print "RESULT:", result
        self._shot_plan.publish(result.shot)

    def extract_planned_bridge_pose(self, ud, goal):
        return PlaceBridgeGoal(down = True,
                               pose = ud.planned_shot.bridge_pose)

    def run(self):
        rospy.init_node('play_fixed')
    
        sm = StateMachine(outcomes = ['succeeded', 'aborted', 'preempted'])
    
        with sm:
            StateMachine.add('OBSERVE_TABLE',
                             SimpleActionState('rviz_get_table_state', GetTableStateAction,
                                               goal = GetTableStateGoal(), result_cb = self.print_result,
                                               result_slots_map = {'state': 'observed_table_state'}),
                             {'succeeded': 'PLAN_SHOT'})
    
            StateMachine.add('PLAN_SHOT',
                             SimpleActionState('plan_shot', PlanShotAction, result_cb = self.publish_plan_shot_result,
                                               goal_slots_map = {'observed_table_state': 'state'},
                                               result_slots_map = {'shot': 'planned_shot'}),
                             {'succeeded': 'succeeded'})
    
        # Run state machine introspection server
        intro_server = IntrospectionServer('play_fixed_smach',sm,'/PLAY')
        intro_server.start()
    
        # Run state machine action server 
        asw = ActionServerWrapper(
                'play_fixed', PlayAction, sm,
                succeeded_outcomes = ['succeeded'],
                aborted_outcomes = ['aborted'],
                preempted_outcomes = ['preempted'])
        asw.run_server()
        
        rospy.spin()
        intro_server.stop()

if __name__ == '__main__':
    play_fixed = PlayFixed()
    play_fixed.run()
