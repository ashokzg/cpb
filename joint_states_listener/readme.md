Use the service to move the arms

Pre-start:
---------
roslaunch pr2_billiards_world billiards_world.launch
rosrun joint_states_listener joint_states_listener.py 

Usage:
-----
Command:
rosrun simple_trajectory moveOneJoint.py 1 2 3 0.5

Description:

rosrun simple_trajectory moveOneJoint.py ARMSIDE JOINTINDEX JOINTANGLE JOINTVELOCITY 
ARMSIDE:
0: Right
1: Left

JOINTINDEX:
0: Shoulder pan (positive away from robot)
1: Shoulder lift (positive towards ground)
2: Shoulder roll (positive counterclockwise for viewer in front of PR2)
3: Elbow (positive stretches)
4: Forearm roll (positive counterclockwise for viewer in front of PR2)
5: Wrist pan (not possible to define)
6: Wrist roll (not possible to define)


JOINTANGLE: 
Angle in radians

JOINTVELOCITY:
Angluar Velocity
