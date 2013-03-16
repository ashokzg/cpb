#!/usr/bin/env python
# 
#  testing, do not use 
# 
#
# spawning things "in sequence"
#
# This quick hack waits for a topic to be published (presumably by the model we are waiting for)
# then it accesses the factory service exposed by gazebo_ros_factory plugin for spawning models
#
# a better way to do this is by specifying
# a list of model spawning dependencies, i.e. don't spawn cup unless table is present
# to do this, we must make gazeob expose
#   * service call to access all models
#   * service call to access info about each model
#   * service call to ping, create, kill, modify (pose) models
# I guess one can construct this similar to rostopic or rosnode

import roslib, time
roslib.load_manifest('pr2_billiards_sim')

import rospy, sys
import string
import math

from gazebo_plugins import gazebo_plugins_interface
from gazebo_plugins.msg import GazeboModel
from geometry_msgs.msg import Pose, Point, Quaternion
import tf.transformations as tft

wait_topic_initialized = False
def usage():
    print '''Commands:
    wait_spawn.py <param name> <model name> <namespace> <topic name> <initial pose: x y z R P Y>
    '''
    sys.exit(1)

def waitForTopic(any):
    global wait_topic_initialized
    wait_topic_initialized = True

if __name__ == "__main__":
    global wait_topic_initialized
    print len(sys.argv) 
    if len(sys.argv) < 11:
        print usage()

    rospy.init_node("spawn_wait", anonymous=True)


    param_name = sys.argv[1]
    model_name = sys.argv[2]
    namespace  = sys.argv[3]
    topic_name = sys.argv[4]
    lx = float(sys.argv[5])
    ly = float(sys.argv[6])
    lz = float(sys.argv[7])
    rx = float(sys.argv[8])
    ry = float(sys.argv[9])
    rz = float(sys.argv[10])
    q = tft.quaternion_from_euler(rx,ry,rz)
    model_msg = GazeboModel(model_name,param_name,GazeboModel.URDF_PARAM_NAME,namespace,Pose(Point(lx,ly,lz),Quaternion(q[0],q[1],q[2],q[3])))

    # wait for p3d if user requests
    print "waiting for topic ",topic_name," before spawning ",model_name," from param ",param_name," in namespace ",namespace,rx, ry, rz
    rospy.Subscriber(topic_name, rospy.AnyMsg, waitForTopic)
    while not wait_topic_initialized:
      time.sleep(0.5)

    print "spawning..."
    # call service to spawn model
    success = gazebo_plugins_interface.load_model(model_msg)
    if success:
      print "spawning ",model_name," success!"
    else:
      print "spawning ",model_name," failed, see console for details."

