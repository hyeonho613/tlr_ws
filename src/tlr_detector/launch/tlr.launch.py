from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    v4l2_camera_pkg = get_package_share_directory('v4l2_camera')
    ultralytics_ros_pkg = get_package_share_directory('ultralytics_ros')
    tlr_detector_pkg = get_package_share_directory('tlr_detector')

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(v4l2_camera_pkg, 'launch', 'v4l2_camera.launch.py')),
        launch_arguments={
            'camera_name': 'front',
            'image_topic': 'image_raw',
            'v4l2_camera_param_path': '/home/sws/.ros/camera_info/camera_params.yaml',
            'rate_diagnostics_param_path': '/home/sws/.ros/camera_info/rate_diagnostics.yaml',
            'camera_info_url': 'file:///home/sws/.ros/camera_info/gmsl2-usb3.0_conversion_kit.yaml',
            'hardware_id': '/dev/video0',
            'use_sensor_data_qos': 'True'
        }.items()
    )

    yolo_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(ultralytics_ros_pkg, 'launch', 'tracker.launch.xml')),
        launch_arguments={
            'debug': 'false'
        }.items()
    )

    tlr_detector_node = Node(
        package='tlr_detector',
        executable='tlr_detector_node',
        name='tlr_detector_node',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        camera_launch,
        yolo_launch,
        tlr_detector_node,
    ])
