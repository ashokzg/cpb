rosservice call gazebo/set_model_state '{model_state: {model_name: pool_ball_0, pose: { position: { x: 1.5 , z: 1 } } } }'
rosservice call gazebo/set_model_state '{model_state: {model_name: pool_ball_1, pose: { position: { x: 1.8 , y: 0.03, z: 1.0 } } } }'
rosservice call gazebo/apply_body_wrench '{body_name: "pool_ball_0::ball_0", wrench: { force: { x: 10 } }, start_time: 1000000000, duration: 100000000}'
