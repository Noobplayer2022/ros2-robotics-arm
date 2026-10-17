from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

from launch.actions import TimerAction

def generate_launch_description():
    pkg_share = get_package_share_directory('combined_pkg')
    urdf_file = os.path.join(pkg_share, 'urdf', 'combine.urdf')
    controllers_file = os.path.join(pkg_share, 'config', 'ros2_controllers.yaml')

    # robot_description
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # Spawn robot in Gazebo
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'combined_robot',
            '-topic', 'robot_description',
            '-z', '0.0'
        ],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/magnet/attach@std_msgs/msg/Bool]gz.msgs.Boolean',
            '/magnet/status@std_msgs/msg/Bool[gz.msgs.Boolean',
        ],
        output='screen'
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Spawner برای joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    # Spawner برای arm_controller
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--param-file', controllers_file
        ],
        output='screen',
    )

    # بعد از اتمام spawn، کنترلرها رو با کمی تأخیر لود کن
    delayed_joint_state_broadcaster = TimerAction(
        period=3.0,
        actions=[joint_state_broadcaster_spawner]
    )


    cube_spawner = TimerAction(
        period=5.0,  # صبر تا world و ربات آماده شوند
        actions=[
            Node(
                package='combined_pkg',
                executable='cube_spawner.py',
                name='cube_spawner',
                output='screen',
            )
        ],
    )


    spawner_gripper = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', os.path.join(pkg_share, 'config', 'ros2_controllers.yaml'),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    delayed_arm_controller = TimerAction(
        period=5.0,
        actions=[arm_controller_spawner]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn,
        bridge,
        rviz,
        delayed_joint_state_broadcaster,
        delayed_arm_controller,
        cube_spawner,
        spawner_gripper
    ])