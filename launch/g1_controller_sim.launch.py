from launch import LaunchDescription
import os
import tempfile
import yaml

from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_rviz(context):
    config_path = PathJoinSubstitution([
        FindPackageShare('g1_controller'), 'config', 'rviz', 'config.rviz'
    ]).perform(context)
    cleanup = []
    if IfCondition(LaunchConfiguration('sam3')).evaluate(context):
        with open(config_path) as stream:
            config = yaml.safe_load(stream)
        camera = next(item for item in config['Visualization Manager']['Displays']
                      if item.get('Name') == 'Camera')
        overlay = next(item for item in camera['Displays'] if item.get('Name') == 'SAM3 Overlay')
        overlay.update(Enabled=True, Value=True)
        positions = next(item for item in camera['Displays'] if item.get('Name') == 'SAM3 Positions')
        positions.update(Enabled=True, Value=True)
        with tempfile.NamedTemporaryFile(mode='w', prefix='g1-sam3-rviz-', suffix='.rviz',
                                         delete=False) as stream:
            yaml.safe_dump(config, stream)
            config_path = stream.name

        def remove_config(_context):
            if os.path.exists(config_path):
                os.unlink(config_path)
            return []

        cleanup = [RegisterEventHandler(OnShutdown(
            on_shutdown=[OpaqueFunction(function=remove_config)]))]
    return cleanup + [Node(package='rviz2', executable='rviz2', name='rviz2',
                           arguments=['-d', config_path])]


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
        DeclareLaunchArgument(
            'sam3', default_value='false',
            description='Enable text-prompt SAM 3 perception on the shared camera stream'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('g1_controller'), 'launch', 'g1_sam3.launch.py'
            ])),
            condition=IfCondition(LaunchConfiguration('sam3')),
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

        OpaqueFunction(function=launch_rviz, condition=IfCondition(rviz)),
    ])
