#!/usr/bin/env python

import roslib; roslib.load_manifest('fastfiz')
import rospy
import time

from fastfiz_msgs.srv import SimulateShot, SimulateShotRequest, SimulateShotResponse
from billiards_msgs.msg import TableState, BallState 

if __name__ == "__main__":
    rospy.init_node("shot_test")
    
    ss = rospy.ServiceProxy("simulate_shot", SimulateShot, persistent=True)
    
    pub = rospy.Publisher("simulated_shot", TableState, latch=True)
    
    vel = 0.5
    phi = 0.0
    while (True):
        req = SimulateShotRequest()
        ball = BallState()
        ball.id = 0
        ball.point.point.x = 0.5
        ball.point.point.y = 0.5
    
        req.state.balls.append(ball)
        
        ball2 = BallState()
        ball2.id = 1
        ball2.point.point.x = 1.0
        ball2.point.point.y = 0.5
        req.state.balls.append(ball2)
    
        req.shot.v = vel
        req.shot.theta = 1.5
        req.shot.phi = phi
        
        res = ss.call(req)
        pub.publish(res.state)
        #print vel, phi
        print res.events
        
        phi += 0.1
        
        #vel += 0.01
        if (vel > 3.0):
            vel = 0.5
            
        time.sleep(0.01)