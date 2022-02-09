#!/usr/bin/env python3

"""Test gazebo_plugins for pedsim."""

import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import launch_ros



def generate_launch_description():
    # Get the launch directory
    pedsim_dir = get_package_share_directory('pedsim_gazebo_plugin')

    # bringup_dir = get_package_share_directory('neuronbot2_description')
    pkg_share = get_package_share_directory('mii_bot_description')
    urdf = os.path.join(pkg_share, 'src/description/mii_bot_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/mii_bot_config.rviz')

    launch_dir = os.path.join(pedsim_dir, 'launch')
    gazebo_dir = get_package_share_directory('gazebo_ros')
    gazebo_launch_dir = os.path.join(gazebo_dir, 'launch')


    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Launch configuration variables specific to simulation
    use_simulator = LaunchConfiguration('use_simulator')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    gazebo_model_path = os.path.join(get_package_share_directory('pedsim_gazebo_plugin'), 'models')
    gazebo_model_path += ':' + os.path.join(get_package_share_directory('mii_bot_description'), 'src')

    print(gazebo_model_path)

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += ":" + gazebo_model_path
    else :
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path

    print(os.environ['GAZEBO_MODEL_PATH'])

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient)')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pedsim_dir, 'worlds', 'house.world'),
        description='Full path to world model file to load')

    # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
        condition=IfCondition(use_simulator),
        cmd=['gzserver', 
        '-s', 'libgazebo_ros_init.so',
        '-s', 'libgazebo_ros_factory.so',
        world],
        cwd=[launch_dir], output='screen')


    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])),
        cmd=['gzclient'],
        cwd=[launch_dir], output='screen')
    
    agent_spawner_cmd = Node(
        package='pedsim_gazebo_plugin',
        executable='spawn_pedsim_agents.py',
        name='spawn_pedsim_agents',
        output='screen')

    # urdf = os.path.join(bringup_dir, 'urdf', 'neuronbot2.urdf')

    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=remappings,
        arguments=[urdf])


    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)

    # Add any conditioned actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(agent_spawner_cmd)
    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_robot_state_publisher_cmd)

    return ld
