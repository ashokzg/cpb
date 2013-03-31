Use the service to move the arms  

Pre-start:  
---------  
roslaunch pr2_billiards_world billiards_world.launch  
rosrun joint_states_listener joint_states_listener.py  
roslaunch simple_trajectory simple_trajectory.launch  

Usage:  
-----  
Command: Move one joint  
---------------------------  
rosrun simple_trajectory moveOneJoint.py ArmSide JointIndex JointAngle JointVelocity
  
Description:  
e.g.: rosrun simple_trajectory moveOneJoint.py 1 2 3 0.5  

ArmSide:  
0: Right  
1: Left  
  
JointIndex:  
0: Shoulder pan (positive away from robot)  
1: Shoulder lift (positive towards ground)  
2: Shoulder roll (positive counterclockwise for viewer in front of PR2)  
3: Elbow (positive stretches)  
4: Forearm roll (positive counterclockwise for viewer in front of PR2)  
5: Wrist pan (not possible to define)  
6: Wrist roll (not possible to define)  


JointAngle:   
Angle in radians  

JointVelocity:  
Angluar Velocity  


Command: Move multiple joints  
---------------------------  
rosservice call move_multiple_joints ArmSide [JOINT_0_ANGLE, JOINT_1_ANGLE, ... JOINT_6_ANGLE]  

Description:  
e.g.: rosservice call move_multiple_joints 0 [0.5, -1, 3, -0.5, 0, -1, 2]  
ArmSide: same as above  
JOINT_x_ANGLE: angles for the joints described above  

  
Command: Take a shot  
---------------------------  
rosservice take_shot Take  

  
Desciption:  
e.g.: rosservice take_shot 0  
Service takes the shot described by the parameter simple_trajectory/ShotStart/j0..j6 


TAKE: 
0 => Take the shot   1 => Retreive cue stick to start position

Useful Link:
------------
1) Uderstanding the planning scene:
https://kforge.ros.org/Sushi/trac/wiki/Planning/PlanningScene

2) Understanding the collision map: 
https://kforge.ros.org/Sushi/trac/wiki/Perception/CollisionMap


