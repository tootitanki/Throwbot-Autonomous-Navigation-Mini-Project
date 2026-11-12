"""One trial (or an interactive run) of the sensor study.
Localization is FIXED and independent of the sensor under test: Gazebo odometry (near-ideal) with an identity map->odom and a
ground-truth static map of the room. The tested sensor feeds ONLY the obstacle layers of the Nav2 costmaps."""
import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from throwbot_sensors import sensor_cfg


def setup(context):
    pkg = share('throwbot_sensors')
    L = lambda n: LaunchConfiguration(n).perform(context)
    sensor = L('sensor')
    gen = sensor_cfg.gen_dir()
    nav2_yaml = sensor_cfg.nav2_params_file(sensor, gen)
    bridge_yaml = sensor_cfg.bridge_file(sensor, gen)
    urdf = ParameterValue(Command(['xacro ', os.path.join(pkg, 'urdf', 'throwbot.urdf.xacro'), ' ' + sensor_cfg.xacro_args(sensor)]), value_type=str)
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r ' + os.path.join(pkg, 'worlds', 'test_room.sdf') + ' ' + L('gz_extra')}.items())
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share('nav2_bringup'), 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': 'true', 'params_file': nav2_yaml, 'autostart': 'true'}.items())
    mapnodes = [
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[{'yaml_filename': os.path.join(pkg, 'maps', 'room.yaml'), 'use_sim_time': False}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map',
             parameters=[{'autostart': True, 'node_names': ['map_server'], 'use_sim_time': False}]),
    ]
    nodes = [
        gz,
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': urdf, 'use_sim_time': True}]),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-topic', 'robot_description', '-name', 'throwbot', '-x', '-4.5', '-y', '0.0', '-z', '0.02']),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': bridge_yaml, 'use_sim_time': True}]),
        Node(package='tf2_ros', executable='static_transform_publisher', arguments=['4.5', '0', '0', '0', '0', '0', 'map', 'odom'],
             parameters=[{'use_sim_time': True}]),
        nav2,
        Node(package='throwbot_sensors', executable='obstacle_mover', output='screen',
             parameters=[{'use_sim_time': True, 'speed': float(L('speed')), 'seed': int(L('seed')),
                          'nominal_arrival': float(L('nominal_arrival')), 'jitter': float(L('jitter'))}]),
    ]
    if sensor == 'ultrasonic':
        nodes.append(Node(package='throwbot_sensors', executable='range_from_scan', output='screen',
                          parameters=[{'use_sim_time': True}]))
    if L('run_goal') == 'true':
        nodes.append(Node(package='throwbot_sensors', executable='study_logger', output='screen',
                          parameters=[{'use_sim_time': True, 'sensor': sensor, 'speed': float(L('speed')),
                                       'seed': int(L('seed')), 'out_dir': L('out_dir'), 'run_id': L('run_id')}]))
    if L('rviz') == 'true':
        nodes.append(Node(package='rviz2', executable='rviz2', arguments=['-d', L('rviz_config') or os.path.join(pkg, 'config', 'study.rviz')],
                          parameters=[{'use_sim_time': True}]))
    # map server comes up first, alone, so its lifecycle activation is not starved by Gazebo start-up
    return mapnodes + [TimerAction(period=8.0, actions=nodes)]


def generate_launch_description():
    pkg = share('throwbot_sensors')
    args = [DeclareLaunchArgument(n, default_value=d) for n, d in [
        ('sensor', 'lidar'), ('speed', '0.5'), ('seed', '1'), ('nominal_arrival', '11.0'), ('jitter', '1.0'),
        ('gz_extra', ''), ('rviz', 'false'), ('rviz_config', ''), ('run_goal', 'true'),
        ('out_dir', os.path.expanduser('~/throwbot_sensor_results')), ('run_id', 'manual')]]
    return LaunchDescription(args + [
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', os.path.join(pkg, 'worlds')),
        OpaqueFunction(function=setup)])
