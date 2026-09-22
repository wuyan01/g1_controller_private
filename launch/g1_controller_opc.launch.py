from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    simulation = LaunchConfiguration('simulation')
    network = LaunchConfiguration('network')
    pose_source_odom = LaunchConfiguration('pose_source_odom')
    rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'simulation',
            default_value='false',
            description='Simulation mode'
        ),
        DeclareLaunchArgument(
            'network',
            default_value='eno1',
            description='Network interface'
        ),
        DeclareLaunchArgument(
            'pose_source_odom',
            default_value='/g1_controller/lidar/odom',
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
                'simulation': simulation,
                'network': network,
                'pose_source_odom': pose_source_odom,
                'rviz': rviz,
                'camera': LaunchConfiguration('camera'),
                'sam3': LaunchConfiguration('sam3')
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('livox_ros_driver2'),
                    'launch_ROS2',
                    'msg_MID360_launch.py' 
                ])
            ])
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('fast_lio'),
                    'launch',
                    'mapping.launch.py'
                ])
            ])
        ),
        
    ])
