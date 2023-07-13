import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('solais_bringup'), 'launch'))
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node
from launch.actions import TimerAction, Shutdown
from launch import LaunchDescription
from launch.substitutions import Command

node_params = os.path.join(
    get_package_share_directory('solais_bringup'), 'configs', 'node_params.yaml')

launch_params = yaml.safe_load(open(os.path.join(
    get_package_share_directory('solais_bringup'), 'configs', 'launch_params.yaml')))

robot_description = Command(['xacro ', os.path.join(
    get_package_share_directory('solais_description'), 'urdf', 'infantry_gimbal.urdf.xacro'),
    ' xyz:=', launch_params['odom2camera']['xyz'], ' rpy:=', launch_params['odom2camera']['rpy']])


robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': robot_description,
                 'publish_frequency': 1000.0}]
)

def generate_launch_description():
    
    camera_node = ComposableNode(
        package='solais_camera',
        plugin='solais_camera::MindVisionCameraNode',
        name='camera_node',
        parameters=[node_params],
        extra_arguments=[{'use_intra_process_comms': True}]
    )
    
    detector_node = ComposableNode(
        package='solais_auto_aim',
        plugin='solais_auto_aim::ArmorDetectorNode',
        name='armor_detector',
        parameters=[node_params],
        extra_arguments=[{'use_intra_process_comms': True}]
    )
    
    camera_detector_container = ComposableNodeContainer(
        name='camera_detector_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            camera_node,
            detector_node
        ],
        output='both',
        emulate_tty=True,
        ros_arguments=['--ros-args', '--log-level',
                        'armor_detector:='+launch_params['detector_log_level']],
        on_exit=Shutdown(),
    )
    
    # serial_driver_node = Node(
    #     package='rm_serial_driver',
    #     executable='rm_serial_driver_node',
    #     name='serial_driver',
    #     output='both',
    #     emulate_tty=True,
    #     parameters=[node_params],
    #     on_exit=Shutdown(),
    #     ros_arguments=['--ros-args', '--log-level',
    #                    'serial_driver:='+launch_params['serial_log_level']],
    # )

    # delay_serial_node = TimerAction(
    #     period=1.5,
    #     actions=[serial_driver_node],
    # )

    # delay_tracker_node = TimerAction(
    #     period=2.0,
    #     actions=[tracker_node],
    # )

    return LaunchDescription([
        robot_state_publisher,
        camera_detector_container,
        # delay_serial_node,
        # delay_tracker_node,
    ])
