from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    simulation = LaunchConfiguration('simulation')
    network = LaunchConfiguration('network')
    pose_source_odom = LaunchConfiguration('pose_source_odom')

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
        
        Node(
            package='g1_controller',
            executable='g1_controller',
            name='g1_controller',
            namespace='g1_controller',
            output='screen',
            parameters=[{
                'simulation': simulation,
                'network': network
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

        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', PathJoinSubstitution([
        #         FindPackageShare('g1_controller'),
        #         'config',
        #         'rviz',
        #         'config.rviz'
        #     ])]
        # )
    ])
