"""Common RGB-D interface; simulation images come from g1_mujoco_camera."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    simulation = LaunchConfiguration('simulation')
    return LaunchDescription([
        DeclareLaunchArgument('simulation', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])),
            condition=UnlessCondition(simulation),
            launch_arguments={
                'camera_namespace': '',
                'camera_name': 'camera',
                # Do not inherit the controller's unrelated config_file argument.
                'config_file': "''",
                'enable_color': 'true',
                'enable_depth': 'true',
                'rgb_camera.color_profile': '640,480,30',
                'depth_module.depth_profile': '640,480,30',
                'align_depth.enable': 'true',
                'pointcloud.enable': 'true',
                'publish_tf': 'true',
                'base_frame_id': 'link',
            }.items(),
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='sim_camera_optical_frame', condition=IfCondition(simulation),
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--qx', '-0.5', '--qy', '0.5', '--qz', '-0.5', '--qw', '0.5',
                       '--frame-id', 'camera_link',
                       '--child-frame-id', 'camera_color_optical_frame'],
        ),
    ])
