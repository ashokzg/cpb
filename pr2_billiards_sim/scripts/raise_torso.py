#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_billiards_sim')
import rospy
from pr2_controllers_msgs.msg import SingleJointPositionActionGoal
def torso_goal_publisher():
    pub = rospy.Publisher('/torso_controller/position_joint_action/goal', SingleJointPositionActionGoal)
    rospy.init_node('torso_goal_publisher')
    while not rospy.is_shutdown():
        sjpa = SingleJointPositionActionGoal();
        sjpa.goal.position = 0.3;
        pub.publish(sjpa)
        rospy.sleep(1.0)
if __name__ == '__main__':
    try:
        torso_goal_publisher()
    except rospy.ROSInterruptException: pass
