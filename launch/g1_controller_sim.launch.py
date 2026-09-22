from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    simulation = LaunchConfiguration('simulation')
    network = LaunchConfiguration('network')
    pose_source_odom = LaunchConfiguration('pose_source_odom')
    rviz = LaunchConfiguration('rviz')
    config_file = LaunchConfiguration('config_file')

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
            description='Enable the shared simulated/physical RGB-D camera interface'
        ),
        GroupAction(
            condition=IfCondition(LaunchConfiguration('camera')),
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare('g1_controller'), 'launch', 'g1_camera.launch.py'
                ])),
                launch_arguments={'simulation': simulation}.items(),
            )],
        ),
        
        DeclareLaunchArgument(
            'config_file',
            default_value='config.yaml',
            description='Controller config filename under the package config directory, or an absolute path'
        ),
        Node(
            package='g1_controller',
            executable='g1_controller',
            name='g1_controller',
            namespace='g1_controller',
            output='screen',
            parameters=[{
                'simulation': simulation,
                'network': network,
                'config_file': config_file
            }],
            remappings=[
                ('/g1_controller/pose_source/odom', pose_source_odom)
            ]
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('g1_description'),
                    'launch',
                    'g1_description.launch.py'
                ])
            ])
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(rviz),
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('g1_controller'),
                'config',
                'rviz',
                'config.rviz'
            ])]
        )
    ])
