# Tiago_pro

## Pre-requisite:

- Running tiago pro docker container

This is the workspace folder for tiago robot which is mounted inside the docker image for development of Holland Project

NOTE: First create folder named "workspace " in your local PC and then clone this repo inside it and then mount it to tiago pro docker container

Follow the below commands inside the docker container. For some extra debugging commands and tiago pro development refer to the Tiago_pro_Startup.txt file in the docs folder.

1. Gazebo simulation

`ros2 launch tiago_pro_gazebo tiago_pro_gazebo.launch.py is_public_sim:=False use_sim_time:=True world_name:=table_woodenpeg`

2. Moveit

`ros2 launch tiago_pro_moveit_config moveit_rviz.launch.py use_sim_time:=True`

3. Teleop

For manual teleop `ros2 run key_teleop key_teleop`

For sripted teleop `ros2 run teleop_tiago teleop --ros-args -p linear_speed:=0.2 -p linear_distance:=0.8`

You can use `angular_speed` and `angular_distance` parameters as well to rotate the robot.

4. Gripper Attach client

Left Gripper

`ros2 run attach_gazebo_client attach_gazebo   --ros-args   -p reference_frame:=gripper_left_fingertip_right_link   -p model_name:=coke_can_slim   -p x_offset:=-0.045   -p y_offset:=0.05   -p z_offset:=0.0   -p orientation_x:=0.0   -p orientation_y:=0.0   -p orientation_z:=0.7071   -p orientation_w:=1.0`

OR

Right Gripper

`ros2 run attach_gazebo_client attach_gazebo   --ros-args   -p reference_frame:=gripper_right_fingertip_left_link   -p model_name:=coke_can_slim   -p x_offset:=-0.045   -p y_offset:=0.05   -p z_offset:=0.0   -p orientation_x:=0.0   -p orientation_y:=0.0   -p orientation_z:=0.7071   -p orientation_w:=1.0`

and then in another terminal run

To attach: `ros2 service call /attach_control attach_gazebo_interfaces/srv/AttachCommand "{command: 'close'}"`

To detach: `ros2 service call /attach_control attach_gazebo_interfaces/srv/AttachCommand "{command: 'open'}"`

5. Package_name: tiago_pro_base_anchor

This package helps to fix the base to avoid drifts in gazebo simulation. JUst for simulation purpose

`ros2 run tiago_pro_base_anchor base_anchor`

In another terminal call the service

`ros2 service call /base_anchor_control attach_gazebo_interfaces/srv/AttachCommand "{command: 'hold'}"`

`ros2 service call /base_anchor_control attach_gazebo_interfaces/srv/AttachCommand "{command: 'release'}"`

6. Pick_place task

`ros2 run cartesian_mover pick_cube_node`