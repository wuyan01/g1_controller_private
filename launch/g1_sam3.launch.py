"""SAM 3 perception for either camera backend; does not start the controller."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix


def generate_launch_description():
    arguments = [
        ('sam3_prompt', 'chair', 'Initial text prompt; updates arrive on /sam3/prompt'),
        ('sam3_checkpoint', '', 'Local sam3.pt; empty downloads/caches facebook/sam3 weights'),
        ('sam3_device', 'cuda', 'PyTorch device, e.g. cuda, cuda:1, or cpu'),
        ('sam3_rate', '2.0', 'Maximum inference starts per second; old frames are dropped'),
        ('sam3_threshold', '0.5', 'Detection confidence threshold'),
        ('sam3_python', 'python3', 'Python interpreter with ROS and SAM 3 dependencies'),
        ('sam3_image_topic', '/camera/color/image_raw', 'RGB input topic'),
        ('sam3_localize', 'true', 'Estimate masked object positions using aligned depth and TF'),
        ('sam3_target_frame', 'pelvis', 'Frame for 3D object positions at image capture time'),
        ('sam3_benchmark', 'false', 'Print simulation reference-point discrepancies in pelvis'),
        ('sam3_target_body', 'desk_Furniture_Monitor', 'Exact MuJoCo body name for benchmarking'),
    ]
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=description)
          for name, default, description in arguments],
        Node(
            package='g1_controller', executable='sam3_benchmark.py',
            name='sam3_benchmark', output='screen',
            condition=IfCondition(LaunchConfiguration('sam3_benchmark')),
            parameters=[{
                'target_body': ParameterValue(LaunchConfiguration('sam3_target_body'), value_type=str),
                'target_label': ParameterValue(LaunchConfiguration('sam3_prompt'), value_type=str),
            }],
        ),
        Node(
            package='g1_controller', executable='sam3_localization.py',
            name='sam3_localization', output='screen',
            condition=IfCondition(LaunchConfiguration('sam3_localize')),
            parameters=[{'target_frame': ParameterValue(
                LaunchConfiguration('sam3_target_frame'), value_type=str)}],
        ),
        Node(
            package='g1_controller', executable='sam3_perception.py',
            name='sam3_perception', output='screen',
            prefix=[PathJoinSubstitution([FindPackagePrefix('g1_controller'), 'lib',
                                          'g1_controller', 'sam3_python.sh']),
                    ' ', LaunchConfiguration('sam3_python')],
            parameters=[{
                'prompt': ParameterValue(LaunchConfiguration('sam3_prompt'), value_type=str),
                'checkpoint': ParameterValue(LaunchConfiguration('sam3_checkpoint'), value_type=str),
                'device': ParameterValue(LaunchConfiguration('sam3_device'), value_type=str),
                'image_topic': ParameterValue(LaunchConfiguration('sam3_image_topic'), value_type=str),
                'max_rate_hz': ParameterValue(LaunchConfiguration('sam3_rate'), value_type=float),
                'confidence_threshold': ParameterValue(LaunchConfiguration('sam3_threshold'), value_type=float),
            }],
        ),
    ])
