import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory 

def generate_launch_description():
    pedsim_dir = get_package_share_directory('pedsim_gazebo_plugin')
    pkg_share = get_package_share_directory('mii_bot_description')
    default_model_path = os.path.join(pkg_share, 'src/description/mii_bot_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/mii_bot_config.rviz')
    world_path=os.path.join(pedsim_dir, 'worlds', 'house.world')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'mii_bot', '-topic', 'robot_description'],
        output='screen'
    )
    robot_localization_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    agent_spawner_cmd = launch_ros.actions.Node(
        package='pedsim_gazebo_plugin',
        executable='spawn_pedsim_agents.py',
        name='spawn_pedsim_agents',
        output='screen')

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
	    launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),

        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
        robot_localization_node,
        rviz_node
    ])