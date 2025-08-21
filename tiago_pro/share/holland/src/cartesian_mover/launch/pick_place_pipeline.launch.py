from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit, OnProcessStart
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    gazebo_launch_file = os.path.join(
        get_package_share_directory('tiago_pro_gazebo'),
        'launch',
        'tiago_pro_gazebo.launch.py'
    )
    moveit_launch_file = os.path.join(
        get_package_share_directory('tiago_pro_moveit_config'),
        'launch',
        'moveit_rviz.launch.py'
    )

    # Step 1: Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            'is_public_sim': 'False',
            'use_sim_time': 'True',
            'world_name': 'table_woodenpeg'
        }.items()
    )

    # Step 2: MoveIt
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch_file),
        launch_arguments={'use_sim_time': 'True'}.items()
    )

    # Step 3: Teleop with sufficient delay
    teleop = ExecuteProcess(
        cmd=['ros2', 'run', 'teleop_tiago', 'teleop',
             '--ros-args', '-p', 'linear_speed:=0.2', '-p', 'linear_distance:=0.8'],
        output='screen'
    )
    
    teleop_delayed = TimerAction(period=20.0, actions=[teleop])

    # Step 4: Start base anchor service (after teleop completes)
    start_base_anchor = ExecuteProcess(
        cmd=['ros2', 'run', 'tiago_pro_base_anchor', 'base_anchor'],
        output='screen'
    )

    # Step 5: Call base anchor service (2 seconds after base anchor starts)
    call_base_anchor_service = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/base_anchor_control', 
             'attach_gazebo_interfaces/srv/AttachCommand', 
             '{command: hold}'],
        output='screen'
    )

    call_service_delayed = TimerAction(
        period=2.0,
        actions=[call_base_anchor_service]
    )

    # Left gripper attach (2s after base anchor service call)
    attach_left_gripper = TimerAction(
        period=2.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'run', 'attach_gazebo_client', 'attach_gazebo',
                 '--ros-args',
                 '-p', 'reference_frame:=gripper_left_fingertip_right_link',
                 '-p', 'model_name:=coke_can_slim',
                 '-p', 'service_name:=attach_control_left',
                 '-p', 'x_offset:=-0.045',
                 '-p', 'y_offset:=0.05',
                 '-p', 'z_offset:=0.0',
                 '-p', 'orientation_x:=0.0',
                 '-p', 'orientation_y:=0.0',
                 '-p', 'orientation_z:=0.7071',
                 '-p', 'orientation_w:=1.0'],
            output='screen'
        )]
    )

    # Right gripper attach (3s after base anchor service call)
    attach_right_gripper = TimerAction(
        period=3.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'run', 'attach_gazebo_client', 'attach_gazebo',
                 '--ros-args',
                 '-p', 'reference_frame:=gripper_right_fingertip_left_link',
                 '-p', 'model_name:=coke_can_slim_clone',
                 '-p', 'service_name:=attach_control_right',
                 '-p', 'x_offset:=-0.045',
                 '-p', 'y_offset:=0.05',
                 '-p', 'z_offset:=0.0',
                 '-p', 'orientation_x:=0.0',
                 '-p', 'orientation_y:=0.0',
                 '-p', 'orientation_z:=0.7071',
                 '-p', 'orientation_w:=1.0'],
            output='screen'
        )]
    )


    # Step 6: Pick and place node (after base anchor service call completes)
    pick_place_node = ExecuteProcess(
        cmd=['ros2', 'run', 'cartesian_mover', 'pick_cube_node'],
        output='screen'
    )

    # Chain the events
    start_base_anchor_after_teleop = RegisterEventHandler(
        OnProcessExit(
            target_action=teleop,
            on_exit=[start_base_anchor]
        )
    )

    call_service_after_base_anchor = RegisterEventHandler(
        OnProcessStart(
            target_action=start_base_anchor,
            on_start=[call_service_delayed]
        )
    )

    attach_left_gripper_service = RegisterEventHandler(
        OnProcessExit(
            target_action=call_base_anchor_service,
            on_exit=[attach_left_gripper]
        )
    )

    attach_right_gripper_service = RegisterEventHandler(
        OnProcessExit(
            target_action=call_base_anchor_service,
            on_exit=[attach_right_gripper]
        )
    )

    start_pick_place_after_service = RegisterEventHandler(
        OnProcessExit(
            target_action=call_base_anchor_service,
            on_exit=[pick_place_node]
        )
    )

    return LaunchDescription([
        gazebo,
        moveit,
        teleop_delayed,
        start_base_anchor_after_teleop,
        call_service_after_base_anchor,
        attach_left_gripper_service,
        attach_right_gripper_service,
        start_pick_place_after_service
    ])