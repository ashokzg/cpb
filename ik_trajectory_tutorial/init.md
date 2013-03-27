rosservice call /left_execute_cartesian_ik_trajectory -- "{header: { frame_id: /base_link}, poses: [{position: [0.349, 0.660, 1.002], orientation:  [-0.000, 0.707, 0.000, 0.707]}]}"

rosservice call /execute_cartesian_ik_trajectory -- "{header: { frame_id: /base_link}, poses: [{position: [0.120, -0.290, 0.430], orientation:[-0.000, -0.111, -0.000, 0.994]}]}"


Right:
curPosition: [-1.8425060510635376,1.2198034524917603,-2.2416832447052,-1.7119375467300415,1.6620752811431885,-0.41495949029922485,-1.3718557357788086]  


Left:  
curPosition: [1.3493558168411255,-0.30538955330848694,1.2200419902801514,-1.244804859161377,1.3725829124450684,-1.989667534828186,-3.133694648742676]  
