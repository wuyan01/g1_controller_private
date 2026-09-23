"""Opt-in native ScaleBFM actor. Existing launch profiles keep their policies."""
import atexit
from pathlib import Path
import tempfile

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def controller(context):
    model = LaunchConfiguration('scalebfm_model').perform(context)
    if model not in ('m', 'xl'):
        raise ValueError('scalebfm_model must be m or xl')
    if LaunchConfiguration('simulation').perform(context).lower() != 'true':
        raise ValueError('ScaleBFM deployment profile is simulation-only until separately qualified on hardware')
    if LaunchConfiguration('network').perform(context) != 'lo':
        raise ValueError('ScaleBFM simulation profile requires network:=lo')
    if LaunchConfiguration('pose_source_odom').perform(context) != '/g1_controller/sport_mode/odom':
        raise ValueError('ScaleBFM currently requires native sport-mode pelvis odometry')
    wrists = LaunchConfiguration('enable_wrist_targets').perform(context)
    if wrists not in ('true', 'false'):
        raise ValueError('enable_wrist_targets must be exactly true or false')

    def include(config_file):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('g1_controller'), 'launch', 'g1_controller_sim.launch.py'])),
            launch_arguments={
            'config_file': config_file,
            **{name: LaunchConfiguration(name) for name in
               ('simulation', 'network', 'steering_ros_enabled', 'pose_source_odom', 'camera', 'rviz', 'sam3')},
            }.items())

    profile_name = f'config_scalebfm_{model}.yaml'
    if wrists == 'false':
        return [include(profile_name)]

    # Resolve the selected installed overlay, and change only this leaf. Copy
    # the mappings along its path so YAML aliases cannot modify another section.
    share = Path(FindPackageShare('g1_controller').perform(context))
    profile = yaml.safe_load((share / 'config' / profile_name).read_text())
    if (not isinstance(profile, dict) or not isinstance(profile.get('FSM'), dict)
            or not isinstance(profile['FSM'].get('ScaleBFM'), dict)):
        raise ValueError('Installed ScaleBFM profile must contain FSM.ScaleBFM settings')
    modified = {**profile, 'FSM': {
        **profile['FSM'], 'ScaleBFM': {
            **profile['FSM']['ScaleBFM'], 'enable_wrist_targets': True,
        },
    }}
    temporary_path = None

    def cleanup(*_unused):
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)
        atexit.unregister(cleanup)
        return []

    try:
        with tempfile.NamedTemporaryFile(mode='w', prefix=f'scalebfm_{model}_wrists_',
                                         suffix='.yaml', delete=False) as stream:
            temporary_path = Path(stream.name).resolve()
            yaml.safe_dump(modified, stream, sort_keys=False)
        atexit.register(cleanup)  # Also covers launch-service startup exceptions.
        shutdown = RegisterEventHandler(OnShutdown(on_shutdown=cleanup))
        exited = RegisterEventHandler(OnProcessExit(
            target_action=lambda action: isinstance(action, Node)
            and action.node_package == 'g1_controller'
            and action.node_executable == 'g1_controller',
            on_exit=cleanup))
        # Register cleanup before expanding the included launch. Other children
        # (camera/RViz) exiting must not delete a config the controller may need.
        return [shutdown, exited, include(str(temporary_path))]
    except BaseException:
        cleanup()
        raise


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scalebfm_model', default_value='m'),
        DeclareLaunchArgument('simulation', default_value='true'),
        DeclareLaunchArgument('network', default_value='lo'),
        DeclareLaunchArgument('steering_ros_enabled', default_value='false'),
        DeclareLaunchArgument('enable_wrist_targets', default_value='false',
                              description='Enable experimental native-actor wrist pose commands'),
        DeclareLaunchArgument('pose_source_odom', default_value='/g1_controller/sport_mode/odom'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('sam3', default_value='false'),
        OpaqueFunction(function=controller),
    ])
