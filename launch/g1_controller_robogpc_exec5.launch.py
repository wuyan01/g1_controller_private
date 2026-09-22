from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'simulation',
            default_value='true',
            description='Simulation mode'
        ),
        DeclareLaunchArgument(
            'network',
            default_value='lo',
            description='Network interface'
        ),
        DeclareLaunchArgument(
            'pose_source_odom',
            default_value='/g1_controller/sport_mode/odom',
            description='Topic for pose source odometry'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz with the g1_controller config'
        ),
        DeclareLaunchArgument(
            'camera', default_value='true',
            description='Enable the shared RGB-D camera interface'
        ),
        DeclareLaunchArgument(
            'sam3', default_value='false',
            description='Enable SAM 3 perception; accepts sam3_prompt/checkpoint/device/rate/python'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('g1_controller'),
                    'launch',
                    'g1_controller_sim.launch.py'
                ])
            ]),
            launch_arguments={
                'config_file': 'config_robogpc_exec5.yaml',
                'simulation': LaunchConfiguration('simulation'),
                'network': LaunchConfiguration('network'),
                'pose_source_odom': LaunchConfiguration('pose_source_odom'),
                'rviz': LaunchConfiguration('rviz'),
                'camera': LaunchConfiguration('camera'),
                'sam3': LaunchConfiguration('sam3'),
            }.items()
        ),
    ])
