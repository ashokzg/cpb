#!/usr/bin/env python

import roslib; roslib.load_manifest('billiards_planner')
import rospy

import math
import numpy
import optparse
import sys
import time

from tf import transformations
import actionlib
from fastfiz_msgs.msg import ShotParams
from fastfiz_msgs.srv import SimulateShot, SimulateShotRequest, SimulateShotResponse
from billiards_msgs.msg import TableState, BallState, ShotPlan, PlanShotAction, PlanShotResult, Constants
from math import sqrt

class Line(object):
    def __init__(self, x1, y1, x2, y2):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        
    def __repr__(self):
        return '%s,%s %s,%s'%(self.x1, self.y1, self.x2, self.y2)
        
class Point(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y
        
def clamp(min_val, val, max_val):
    return max(min_val, min(max_val, val))

def line_point_distance(line, p, use_segments=True):
    m = (line.x2 - line.x1, line.y2 - line.y1)
    m_dot_m = (m[0] * m[0]) + (m[1] * m[1])
    if abs(m_dot_m) < 1e-10:
        m_dot_m = 1e-10

    x_sub_b = (p.x - line.x1, p.y - line.y1)
    m_dot_x_sub_b = (m[0] * x_sub_b[0]) + (m[1] * x_sub_b[1])

    t0 = m_dot_x_sub_b / m_dot_m
    if use_segments:
        t0 = clamp(0.0, t0, 1.0)

    line_p = (line.x1 + t0 * m[0], line.y1 + t0 * m[1])
    v = (p.x - line_p[0], p.y - line_p[1])

    return (line_p, sqrt(v[0] * v[0] + v[1] * v[1]))

def is_point_in_rect(px, py, x1, y1, x2, y2):
   xmin, xmax, ymin, ymax = min(x1, x2), max(x1, x2), min(y1, y2), max(y1, y2)
   return px >= xmin and px <= xmax and py >= ymin and py <= ymax

class MCPlanner(object):
    def __init__(self):
        self._plan_shot_server      = actionlib.SimpleActionServer('plan_shot', PlanShotAction, self.plan_shot)
        self._simulate_shot_service = rospy.ServiceProxy('simulate_shot', SimulateShot, persistent=True)
        self._shot_plan             = rospy.Publisher('shot_plan', ShotPlan)
        self._table_state_pub       = rospy.Publisher('simulated_table_state', TableState)

        self._vel_min  = 1.40
        self._vel_max  = 1.41
        self._vel_step = 0.1

        self._phi_step = 0.1

        self._azimuth = 3.0    # azumith of 3 degrees
        
        self._table_state_pub2 = rospy.Publisher("mc_planner_table_state", TableState)

    def shot_plan_from_params(self, table_state, params):
        (bridge_pose, base_pose) = self.calculate_bridge_and_base_pose(table_state, params.phi)
        bridge_pose_pos, bridge_pose_orient = bridge_pose
        base_pose_pos, base_pose_orient = base_pose
        
        # Construct the ShotPlan message
        shot = ShotPlan()
        shot.bridge_pose.header.stamp = rospy.Time.now()
        shot.bridge_pose.header.frame_id = '/table'
        shot.bridge_pose.pose.position.x = bridge_pose_pos[0]
        shot.bridge_pose.pose.position.y = bridge_pose_pos[1]
        shot.bridge_pose.pose.orientation.x = bridge_pose_orient[0]
        shot.bridge_pose.pose.orientation.y = bridge_pose_orient[1]
        shot.bridge_pose.pose.orientation.z = bridge_pose_orient[2]
        shot.bridge_pose.pose.orientation.w = bridge_pose_orient[3]
        shot.base_pose.header.stamp = rospy.Time.now()
        shot.base_pose.header.frame_id = '/table'
        shot.base_pose.pose.position.x = base_pose_pos[0]
        shot.base_pose.pose.position.y = base_pose_pos[1]
        shot.base_pose.pose.position.z = base_pose_pos[2]
        shot.base_pose.pose.orientation.x = base_pose_orient[0]
        shot.base_pose.pose.orientation.y = base_pose_orient[1]
        shot.base_pose.pose.orientation.z = base_pose_orient[2]
        shot.base_pose.pose.orientation.w = base_pose_orient[3]
        return shot
        
    def get_best_shot(self, table_state, angle_min, angle_max, vel_step, phi_step):
        # Brute force search over velocity/phi space
        possible_shots = []
        shot_scores = []
        shot_results = {}
        shot_tables = {}
        i = 0
        for v in numpy.arange(self._vel_min, self._vel_max, vel_step):
            for phi in numpy.arange(angle_min, angle_max, phi_step):
                shot = ShotParams()
                shot.theta = self._azimuth
                shot.v     = v
                shot.phi   = phi
                
                if not self.is_shot_feasible(table_state, shot.phi):
                    continue
                
                possible_shots.append((shot, i))
                i += 1

        print 'Searching over %d possible shots...' % len(possible_shots)

        for shot, index in possible_shots:
            try:
                if rospy.is_shutdown():
                    exit()

                # Simulate the shot
                result = self.simulate_shot(table_state, shot)

                self._shot_plan.publish(self.shot_plan_from_params(table_state, shot))
                self._table_state_pub.publish(result.state)

                # Score it
                score = self.score_shot(table_state, shot, result.state, result.events)

                #print 'theta=%.2f v=%.2f phi=%.1f score=%3d' % (shot.theta, shot.v, shot.phi, score)

                # Store the results
                shot_scores.append((score, shot, index))
                shot_results[shot] = score
                shot_tables[shot] = result.state
            except rospy.ServiceException, ex:
                print str(ex)

        shot_scores = self.increase_wide_angle_shot_scores(shot_scores)

        # Rank the results
        shot_scores.sort()

        # Return the results
        results = list(reversed(shot_scores))
        return results, shot_tables

    def plan_shot(self, goal):
        table_state = goal.state
        
        self._table_state_pub2.publish(table_state)
        
        if (len(table_state.balls) == 0):
            print "No balls in table state"
            self._plan_shot_server.set_aborted(None, "No balls in table state")
            return
        
        if (self.get_cue_ball_pos(table_state) == None):
            print "No cue ball"
            self._plan_shot_server.set_aborted(None, "No cue ball")
            return

        for ball in table_state.balls:
            if not ball.pocketed:
                print '%d (%.2f, %.2f)' % (ball.id, ball.point.point.x, ball.point.point.y)

        if (goal.angle_min == 0.0 and goal.angle_max == 0.0):
            angle_min_deg = 0.0
            angle_max_deg = 360.0
        else:
            angle_min_deg = goal.angle_min * (180.0/math.pi)
            angle_max_deg = goal.angle_max * (180.0/math.pi)
            
        print angle_min_deg, angle_max_deg
        
        (results, shot_tables) = self.get_best_shot(table_state, angle_min_deg, angle_max_deg, self._vel_step, self._phi_step)

        if (len(results) == 0):
            print "No valid shots!"
            self._plan_shot_server.set_aborted(None, "No valid shots found")
        else:
            # No good results
            if (results[0][0] <= 0):
                if (len(table_state.balls) == 1):
                    print "Only found the cue ball"
                else:
                    max_next = min(len(results), 100)
                    print "No good results, simulating next shot for %s results"%(str(max_next))
                    step = int(len(results) / max_next)
                    new_results = []
                    for i, (score, shot, index) in enumerate(results):
                        if (i % step != 0):
                            continue
                        
                        if (score == 0):
                            sub_table = shot_tables[shot] 
                            (sub_results, sub_tables) = self.get_best_shot(sub_table, 0, 360, 0.1, 1.0)
                            
                            if (len(sub_results) == 0):
                                new_results.append((score, shot, index))
                            else:
                                max_score = sub_results[0][0]
                                if (max_score > 0):
                                    new_results.append(((max_score * 0.1), shot, index))
                                else:
                                    new_results.append((score, shot, index))
                        else:
                            new_results.append((score, shot, index))
                    
                    new_results.sort()
                    results = list(reversed(new_results))
            
            print 'Top 10 shots:'
            for i, (score, shot, index) in enumerate(results[:10]):
                rank = i + 1
    
                object_pocketed_before = set([b for b in table_state.balls       if b.pocketed and b.id != 0])
                object_pocketed_after  = set([b for b in shot_tables[shot].balls if b.pocketed and b.id != 0])
                shot_pocketed = object_pocketed_after - object_pocketed_before
    
                if len(shot_pocketed) > 0:
                    print '#%d: %.2f v: %.2f phi: %.2f [pocketed: %s]' % (rank, score, shot.v, shot.phi, ', '.join(sorted([str(b.id) for b in shot_pocketed])))
                else:
                    print '#%d: %.2f v: %.2f phi: %.2f [none pocketed]' % (rank, score, shot.v, shot.phi)
    
            # Choose the best shot
            best_shot = results[0][1]
    
            (bridge_pose, base_pose) = self.calculate_bridge_and_base_pose(table_state, best_shot.phi)
            bridge_pose_pos, bridge_pose_orient = bridge_pose
            base_pose_pos, base_pose_orient = base_pose
    
            # Construct the ShotPlan message
            result = PlanShotResult()
            result.shot = self.shot_plan_from_params(table_state, best_shot)
            
            print "publishing ", shot_tables[best_shot]
            self._table_state_pub.publish(shot_tables[best_shot])
            result.shot.velocity = best_shot.v
    
            print 'Shot plan:'
            print '  bridge_x: %.2f' % bridge_pose_pos[0]
            print '  bridge_y: %.2f' % bridge_pose_pos[1]
            print '  phi:      %.1f' % best_shot.phi
            print '  vel:      %.2f' % best_shot.v
            self._shot_plan.publish(result.shot)
    
            self._plan_shot_server.set_succeeded(result)
        
    def increase_wide_angle_shot_scores(self, shot_scores):
        new_scores = []
        run = 0
        last_phi = -100
        last_score = -100
        for i in xrange(0, len(shot_scores)):
            (score, shot, index) = shot_scores[i]
            #print shot.phi, score
            if (score > 0 and abs(last_phi - shot.phi) < 0.2):
                run += 1
            else:
                if (run == 0):
                    new_scores.append((score, shot, index))
                else:
                    increase = 1
                    diff = 1
                    for j in xrange(i - run, i):
                        (j_score, j_shot, j_index) = shot_scores[j]
                        #print j, i - (run/2), run, "increasing shot ", j_shot.phi, j_score, " by ", increase, " diff ", diff
                        new_scores.append((j_score + increase, j_shot, j_index))
                        if (j >= (i - (run/2))):
                            diff = -1
                            
                        increase = increase + diff
                
                run = 0
                
            last_phi = shot.phi
            last_score = score
            
        return new_scores
            

    def score_shot(self, in_state, shot_params, out_state, events):
        in_balls = dict([(b.id, b) for b in in_state.balls])

        # Score based on pocketed balls
        score = 0
        for ball in out_state.balls:
            if ball.pocketed:
                if ball.id == 0:
                    score -= 10       # cue pocketed: -10
                else:# not in_balls[ball.id].pocketed:
                    score += 1        # ball pocketed: +1

        # Penalize high velocity
        #score -= 0.5 * shot_params.v
        # @todo: take into account rails, first ball hit, distance balls travelled...

        return score

    # Test whether a shot is kinematically feasible (won't bump the base into the table, or the bridge into the rail)
    def is_shot_feasible(self, table_state, phi):
        (bridge_pose, base_pose) = self.calculate_bridge_and_base_pose(table_state, phi)
        bridge_pose_pos, bridge_pose_orient = bridge_pose
        base_pose_pos, base_pose_orient = base_pose

        # Shot isn't feasible if the bridge is within the rail
        bridge_x, bridge_y = bridge_pose_pos[0], bridge_pose_pos[1]
        if bridge_x < 0.0 or bridge_x > Constants.TABLE_LENGTH or bridge_y < 0.0 or bridge_y > Constants.TABLE_WIDTH:
            return False

        table_x0 = -Constants.RAIL_DEPTH
        table_y0 = -Constants.RAIL_DEPTH
        table_x1 = Constants.TABLE_LENGTH + Constants.RAIL_DEPTH
        table_y1 = Constants.TABLE_WIDTH  + Constants.RAIL_DEPTH

        # Shot isn't feasible if the base intersects the table
        base_x, base_y = base_pose_pos[0], base_pose_pos[1]
        
        if (is_point_in_rect(base_x, base_y, table_x0, table_y0, table_x1, table_y1)):
            return False
        
        distances = [line_point_distance(Line(table_x0, table_y0, table_x0, table_y1), Point(base_x, base_y))[1],
                     line_point_distance(Line(table_x0, table_y0, table_x1, table_y0), Point(base_x, base_y))[1],
                     line_point_distance(Line(table_x1, table_y0, table_x1, table_y1), Point(base_x, base_y))[1],
                     line_point_distance(Line(table_x0, table_y1, table_x1, table_y1), Point(base_x, base_y))[1]]
        min_dist = min(distances)
        
        if min_dist <= Constants.ROBOT_RADIUS:
            return False

        return True

    def calculate_bridge_and_base_pose(self, table_state, phi):
        # Calculate bridge pose from cue position and phi
        phi_radians = math.pi * phi / 180.0
        cue_x,    cue_y    = self.get_cue_ball_pos(table_state)
        bridge_x, bridge_y = self.get_bridge_pos(cue_x, cue_y, phi)
        (qx, qy, qz, qw)   = transformations.quaternion_about_axis(phi_radians, (0, 0, 1))

        bridge_pos         = transformations.translation_matrix([bridge_x, bridge_y, 0.0])
        bridge_orient      = transformations.quaternion_matrix([qx, qy, qz, qw])
        bridge_pose        = transformations.concatenate_matrices(bridge_pos, bridge_orient)
        bridge_pose_pos    = transformations.translation_from_matrix(bridge_pose)
        bridge_pose_orient = transformations.quaternion_from_matrix(bridge_pose)

        # Produce base pose via fixed transform from bridge pose
        bridge_to_base_trans = transformations.translation_matrix([Constants.BRIDGE_IN_BASE_X, Constants.BRIDGE_IN_BASE_Y, Constants.BRIDGE_IN_BASE_Z])
        bridge_to_base_rot   = transformations.quaternion_matrix([Constants.BRIDGE_IN_BASE_QX, Constants.BRIDGE_IN_BASE_QY, Constants.BRIDGE_IN_BASE_QZ, Constants.BRIDGE_IN_BASE_QW])
        bridge_to_base       = transformations.concatenate_matrices(bridge_to_base_trans, bridge_to_base_rot)
        base_to_bridge       = transformations.inverse_matrix(bridge_to_base)
        base_pose            = transformations.concatenate_matrices(bridge_pose, base_to_bridge)  # correct order?
        base_pose_pos        = transformations.translation_from_matrix(base_pose)
        base_pose_orient     = transformations.quaternion_from_matrix(base_pose)
        
        return ((bridge_pose_pos, bridge_pose_orient), (base_pose_pos, base_pose_orient))

    # Given the position of the cue ball, and the angle of the cue, determine where to place the bridge
    def get_bridge_pos(self, cue_x, cue_y, phi):
        # @todo: use variable distance (up to Constants.BRIDGE_TO_STRIKE_MAX) if necessary
        bridge_to_strike = Constants.BRIDGE_TO_STRIKE_MIN

        phi_radians = math.pi * phi / 180.0
        bridge_x = cue_x - (bridge_to_strike + Constants.BALL_RADIUS) * math.cos(phi_radians)
        bridge_y = cue_y - (bridge_to_strike + Constants.BALL_RADIUS) * math.sin(phi_radians)

        return (bridge_x, bridge_y)

    # Extract the cue ball position from a TableState
    def get_cue_ball_pos(self, state):
        for ball in state.balls:
            if ball.id == 0:
                return ball.point.point.x, ball.point.point.y

        return None

    # Call fastfiz for the result of the shot
    def simulate_shot(self, table_state, shot_params):
        req = SimulateShotRequest()
        req.state = table_state
        req.shot  = shot_params
        return self._simulate_shot_service.call(req)

if __name__ == '__main__':    
    rospy.init_node('mc_planner')
    planner = MCPlanner()
    rospy.spin()
