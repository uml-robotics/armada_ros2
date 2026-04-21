Files included:
- sim_world.launch.py
- sim_robot.launch.py
- sim_camera.launch.py
- moveit_core.launch.py
- perception_classic.launch.py
- perception_learned.launch.py
- flexbe.launch.py
- full_system.launch.py

Helper module:
- launch_common.py

Suggested usage examples:
1) World only:
   ros2 launch <your_pkg> sim_world.launch.py

2) Robot + camera + MoveIt:
   ros2 launch <your_pkg> full_system.launch.py launch_learned_perception:=False launch_classic_perception:=False

3) UOC + CGN RGBD path:
   ros2 launch <your_pkg> full_system.launch.py \
       launch_cgn_rgbd:=True launch_uoc_rgbd:=True \
       launch_cgn_cloud:=False launch_uoc_cloud:=False \
       launch_graspsam:=False

4) FlexBE on top:
   ros2 launch <your_pkg> full_system.launch.py launch_flexbe:=True
