# Everything is in meters

float32 BALL_RADIUS = 0.028575
float32 TABLE_LENGTH = 2.235
float32 TABLE_WIDTH = 1.120
float32 TABLE_HEIGHT = 0.78
float32 RAIL_DEPTH = 0.14
float32 RAIL_HEIGHT = 0.038
float32 CORNER_POCKET_WIDTH = 0.125
float32 SIDE_POCKET_WIDTH = 0.135
float32 BUMPER_TO_DIAMOND = 0.085

float32 BRIDGE_OFFSET_Z = 0.104
float32 BRIDGE_HEIGHT = 0.112 
float32 BRIDGE_WIDTH = 0.052

float32 BRIDGE_TO_STRIKE_MIN = 0.25
float32 BRIDGE_TO_STRIKE_MAX = 0.33
float32 BRIDGE_THICKNESS = 0.02

float32 ROBOT_RADIUS = 0.40

#Translation: [0.423, 0.758, 0.921]
#- Rotation: in Quaternion [-0.536, -0.452, 0.538, -0.468]
#            in RPY [0.721, 1.549, -1.004]

### Constraint on the offset of the base from the bridge
float32 BRIDGE_IN_BASE_X = 0.428 #0.295 428
float32 BRIDGE_IN_BASE_Y = 0.753 #0.860 753
float32 BRIDGE_IN_BASE_Z = 0.918 #0.710 918 # Use TABLE_HEIGHT - 0.051 (<-- base_link height)

#CMU CPB VALUES
float32 BRIDGE_IN_BASE_QX = -0.539  #539
float32 BRIDGE_IN_BASE_QY = -0.449  #449
float32 BRIDGE_IN_BASE_QZ = 0.540   #540
float32 BRIDGE_IN_BASE_QW = -0.466  #466
# 90 degrees to the base
#float32 BRIDGE_IN_BASE_QX = 0.0
#float32 BRIDGE_IN_BASE_QY = 0.0
#float32 BRIDGE_IN_BASE_QZ = 0.7071
#float32 BRIDGE_IN_BASE_QW = 0.7071
# 100 degrees to the base
#float32 BRIDGE_IN_BASE_QX = 0.0
#float32 BRIDGE_IN_BASE_QY = 0.0
#float32 BRIDGE_IN_BASE_QZ = 0.766
#float32 BRIDGE_IN_BASE_QW = 0.643

float32 CUE_AZIMUTH = 0.0524  # Radians
