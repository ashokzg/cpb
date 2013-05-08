int8                  id                  # id of the ball
                                          # 0-cue ball, 1-15 for each of the 15 balls
bool                  pocketed            # false if ball is on the table
geometry_msgs/PointStamped   point        # location of each ball
int8                  group_id            # if we want to group the ball
