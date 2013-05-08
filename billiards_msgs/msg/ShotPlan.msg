# describes plan for a single shot
BallState ball
geometry_msgs/PoseStamped base_pose                     # where PR2 should "stand"
geometry_msgs/PoseStamped bridge_pose                   # where to put the bridge
                                                        # orientation is: x-forward (towards cue ball)
                                                        #                 z-(up)
                                                        #                 y-robot left
float64                   velocity                      # not used for now

