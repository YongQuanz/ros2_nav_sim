import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ros_gz_bridge.actions import RosGzBridge
from ros_gz_sim.actions import GzServer

def generate_launch_description():

    # Paths
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    gz_spawn_model_launch_source = os.path.join(ros_gz_sim_share, "launch", "gz_spawn_model.launch.py")
    
    pkg_share = FindPackageShare(package='my_robot_description').find('my_robot_description')
    default_model_path = PathJoinSubstitution([pkg_share, "urdf", "sam_bot.sdf"])
    default_rviz_config_path = PathJoinSubstitution([pkg_share, "rviz", "config.rviz"])
    
    bridge_config_path = os.path.join(pkg_share, 'config', 'bridge_config.yaml')
    world_path = os.path.join(pkg_share, 'world', 'my_world.sdf')
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')

    # Launch arguments
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot xacro file'
    )

    rvizconfig_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=default_rviz_config_path,
        description='Absolute path to RVIZ config file'
    )

    # robot_description parameter with xacro
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{"robot_description": robot_description}, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        output='screen'
    )

    # Launch Gazebo and Spawn Model 
    gz_server = GzServer(
        world_sdf_file=world_path,
        container_name='ros_gz_container',
        create_own_container='True',
        use_composition='True',
    )
    ros_gz_bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=bridge_config_path,
        container_name='ros_gz_container',
        create_own_container='False',
        use_composition='True',
    )
    spawn_entity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_spawn_model_launch_source),
        launch_arguments={
            'world': 'my_world',
            'topic': '/robot_description',
            'entity_name': 'sam_bot',
            'z': '0.65',
        }.items(),
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
       
    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'),
        ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen'),
        model_arg,
        rvizconfig_arg,
        robot_state_publisher_node,
        rviz_node,
        gz_server,
        ros_gz_bridge,
        spawn_entity,
        robot_localization_node
    ])