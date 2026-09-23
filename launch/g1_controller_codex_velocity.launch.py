"""Codex controls the existing VELOCITY policy; Start -> A enters it."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arguments = [
        ('simulation', 'true', 'Use simulator feedback'),
        ('network', 'lo', 'Unitree DDS network interface'),
        ('steering_ros_enabled', 'true', 'Allow Codex velocity commands; joystick movement takes over'),
        ('rviz', 'true', 'Show camera, detections, and robot in RViz'),
        ('camera', 'true', 'Enable RGB-D camera interface'),
        ('sam3', 'true', 'Enable SAM3 perception and localization'),
        ('sam3_prompt', 'table', 'Initial perception target'),
    ]
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=description)
          for name, default, description in arguments],
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('g1_controller'), 'launch', 'g1_controller_sim.launch.py'])),
            launch_arguments={
                'config_file': 'config_codex_velocity.yaml',
                **{name: LaunchConfiguration(name) for name, _, _ in arguments},
            }.items(),
        ),
    ])
