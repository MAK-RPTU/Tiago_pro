import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, Shutdown
from launch_ros.actions import Node, SetRemap
from launch.launch_description_sources import PythonLaunchDescriptionSource

# force colorized output for all the nodes
os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"

def generate_launch_description():

    ld = LaunchDescription()

    ld.add_action(SetRemap(src='image', dst='/camera/image_raw'))
    ld.add_action(SetRemap(src='camera_info', dst='/camera/camera_info'))
    ld.add_action(SetRemap(src='/robot_face/look_at', dst='/look_at'))
    ld.add_action(SetRemap(src='/robot_face/tts', dst='/communication_hub/robot_speech'))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('expressive_eyes'), 'launch'), '/expressive_eyes_with_eyes_tf.launch.py']),
        launch_arguments={"general.headless": "True", "general.fg_bitmap": ""}.items(),
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('attention_manager'), 'launch'), '/attention_manager.launch.py'])
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('communication_hub'), 'launch'), '/communication_hub.launch.py']),
        launch_arguments={"activate": "True"}.items(),
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('hri_person_manager'), 'launch'), '/person_manager.launch.py']),
        launch_arguments={"reference_frame": "camera", "robot_reference_frame": "sellion_link"}.items(),
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('hri_face_detect'), 'launch'), '/face_detect.launch.py'])
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('hri_emotion_recognizer'), 'launch'), '/emotion_recognizer.launch.py'])
    ))

    ld.add_action(Node(
        package='gscam',
        executable='gscam_node',
        parameters=[{
            'gscam_config': 'v4l2src device=/dev/video0 ! video/x-raw,framerate=30/1 ! videoconvert',
            'use_sensor_data_qos': True,
            'camera_name': 'camera',
            'camera_info_url': 'package://interaction_sim/config/camera_info.yaml',
            'frame_id': 'camera'
            }]
    ))


    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('hri_visualization'), 'launch'), '/hri_visualization.launch.py'])
    ))

    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.1', '-0.5', '0.5',
                   '-0.5', '0.5', 'sellion_link', 'camera'],
    ))

    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.20', '0', '0',
                   '0', '1', 'base_link', 'sellion_link'],
    ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('knowledge_core'), 'launch'), '/knowledge_core.launch.py'])
    ))

    rqt_cmd = ['rqt',             
            '--perspective-file', 
            os.path.join(
                get_package_share_directory('interaction_sim'),
                'config/simulator.perspective')]

    # Create the ExecuteProcess action
    rqt = ExecuteProcess(
        cmd=rqt_cmd,
        output='log',
        shell=False,
        on_exit=Shutdown()
    )

    ld.add_action(rqt)

    return ld
