#!/usr/bin/env python
import rospy
import roslib; roslib.load_manifest('bridge_mover')
print "fuck you"
f = open('../pr2_billiards_sim/urdf/stick.urdf', 'r')
s = f.read()
rospy.set_param('stick_urdf', s)
