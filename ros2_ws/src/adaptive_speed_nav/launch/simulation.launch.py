"""Gazebo world + robot + controllers + LiDAR bridge (+ navigator if navigate:=true).

    ros2 launch adaptive_speed_nav simulation.launch.py density:=high mode:=adaptive
    ros2 launch adaptive_speed_nav simulation.launch.py density:=high mode:=fixed fixed_speed:=1.0
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('adaptive_speed_nav')
    controllers_yaml = os.path.join(pkg, 'config', 'controllers.yaml')
    with open(os.path.join(pkg, 'urdf', 'mobile_robot.urdf')) as f:
        robot_description = f.read().replace('__CONTROLLERS_YAML__', controllers_yaml)

    args = [
        DeclareLaunchArgument('density', default_value='medium',
                              description='low | medium | high'),
        DeclareLaunchArgument('mode', default_value='adaptive',
                              description='adaptive | fixed'),
        DeclareLaunchArgument('fixed_speed', default_value='1.0'),
        DeclareLaunchArgument('navigate', default_value='true'),
        DeclareLaunchArgument('log_csv', default_value=''),
    ]

    world = [TextSubstitution(text=os.path.join(pkg, 'worlds') + '/cluttered_'),
             LaunchConfiguration('density'), TextSubstitution(text='.sdf')]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [TextSubstitution(text='-r '), *world]}.items())

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
               output='screen')

    # robot starts at (-5, -5) facing +x; odom frame origin = this start pose
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-topic', 'robot_description', '-name', 'mobile_robot',
                            '-x', '-5.0', '-y', '-5.0', '-z', '0.2', '-Y', '0.0'])

    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                  arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                             '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'])

    jsb = Node(package='controller_manager', executable='spawner',
               arguments=['joint_state_broadcaster'], output='screen')
    diff = Node(package='controller_manager', executable='spawner',
                arguments=['diff_drive_controller'], output='screen')
    after_spawn = RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb]))
    after_jsb = RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[diff]))

    navigator = Node(package='adaptive_speed_nav', executable='navigator', output='screen',
                     condition=IfCondition(LaunchConfiguration('navigate')),
                     parameters=[{'use_sim_time': True,
                                  'mode': LaunchConfiguration('mode'),
                                  'fixed_speed': LaunchConfiguration('fixed_speed'),
                                  'log_csv': LaunchConfiguration('log_csv'),
                                  'goal_x': 10.0, 'goal_y': 10.0}])
    # give the controllers time to become active before the navigator starts
    delayed_nav = TimerAction(period=15.0, actions=[navigator])

    return LaunchDescription(args + [gazebo, rsp, bridge, spawn, after_spawn, after_jsb,
                                     delayed_nav])
