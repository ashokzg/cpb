Gripper pose for right arm  

 Translation: [0.120, -0.290, 0.430]
- Rotation: in Quaternion [-0.000, -0.111, -0.000, 0.994]
            in RPY [-0.001, -0.222, -0.000]

 rosservice call /execute_cartesian_ik_trajectory -- "{header: { frame_id: /base_link}, poses: [{position: [0.120, -0.290, 0.430], orientation:[-0.000, -0.111, -0.000, 0.994]}]}"

Service call for left arm IK for reaching the bridge!

 rosservice call /left_execute_cartesian_ik_trajectory -- "{header: { frame_id: /base_link}, poses: [{position: [0.250, 0.840, 0.826], orientation:[0.694, -0.407, -0.478, -0.352]}]}"


TF between bridge and gripper

- Translation: [-0.210, 0.002, 0.002]
- Rotation: in Quaternion [-0.849, 0.000, -0.000, 0.528]
            in RPY [-2.028, 0.000, -0.000]
Use a static TF to publish to dummy frame. IK left gripper to dummy frame!

