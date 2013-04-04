Repo guidelines:
---------------
1. Do "make clean" before committing to repo  
2. Can download entire repo in any directory. So home or opt/ros does not matter  
3. If you think some file is not supposed to go on the repo add it to .gitignore  

Steps to setup simulation:  
-------------------------
1. roslaunch pr2_billiards_sim billiards_world.launch
2. Wait for simulation to get ready. 
3. roslaunch pr2_billiards_sim sim.launch 
4. Wait for stick to be ready in hand
5. rosrun simple_torso simple_torso 
6. Wait for torso to finish rising to max height.
7. roslaunch billiards_navigation navigation.launch
8. roslaunch pr2_navigation_global rviz_move_base.launch ---------To start rviz
9. rosrun base_position publish.py 
10. rosrun base_position base_position.py 
11. Allow the PR2 to navigate to desired position. You might have to kill navigation.
12. rosrun tf tf_echo /base_link /wrist_des   ------------- To get the transform from current base link to desired left wrist position
13. rosservice call /left_execute_cartesian_ik_trajectory -- "{header: { frame_id: /base_link}, poses: [{position: [0.349, 0.661, 0.995], orientation:[-0.000, 0.707, -0.000 0.707]}]}" ---- Replace these values with the tf you receive in step 12. #hack-->(subtract 0.27544 from the Z-value of this transform)
14. rosservice call move_one_joint -- 0 6 -0.5 100   --- take the shot!
 

