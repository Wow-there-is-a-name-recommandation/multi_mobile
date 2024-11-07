import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Simulation time argument
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file for each robot
    pkg_path = os.path.join(get_package_share_directory('turtlebot3_description'))
    
    # Bot 1 URDF with namespace
    xacro_file_1 = os.path.join(pkg_path, 'urdf', 'turtlebot3_waffle_pi.urdf.xacro')
    robot_description_config_1 = xacro.process_file(xacro_file_1, mappings={'robot_name': 'my_bot1'})
    
    # Bot 2 URDF with namespace
    xacro_file_2 = os.path.join(pkg_path, 'urdf', 'turtlebot3_waffle_pi.urdf.xacro')
    robot_description_config_2 = xacro.process_file(xacro_file_2, mappings={'robot_name': 'my_bot2'})
    
    # Bot 3 URDF with namespace
    xacro_file_3 = os.path.join(pkg_path, 'urdf', 'turtlebot3_waffle_pi.urdf.xacro')
    robot_description_config_3 = xacro.process_file(xacro_file_3, mappings={'robot_name': 'my_bot3'})

    '''
    print(robot_description_config_1.toxml())
    print(robot_description_config_2.toxml())
    print(robot_description_config_3.toxml())
    '''
    # Parameters for each robot
    params_bot1 = {'robot_description': robot_description_config_1.toxml(), 'use_sim_time': use_sim_time}
    params_bot2 = {'robot_description': robot_description_config_2.toxml(), 'use_sim_time': use_sim_time}
    params_bot3 = {'robot_description': robot_description_config_3.toxml(), 'use_sim_time': use_sim_time}


    # Robot State Publisher Node for Bot 1
    node_robot_state_publisher_my_bot1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='my_bot1',
        output='screen',
        parameters=[params_bot1]
    )

    # Robot State Publisher Node for Bot 2
    node_robot_state_publisher_my_bot2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='my_bot2',
        output='screen',
        parameters=[params_bot2]
    )

    # Robot State Publisher Node for Bot 3
    node_robot_state_publisher_my_bot3 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='my_bot3',
        output='screen',
        parameters=[params_bot3]
    )

    # Initial cmd_vel publisher
    initial_cmd_vel = Node(
        package='cmd_initial',
        executable='initial_cmd',
        output='screen'
    )

    # Gazebo spawns for each bot
    spawn_bot1 = ExecuteProcess(
        cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
             '-topic', '/my_bot1/robot_description',
             '-entity', 'my_bot1',
             '-x', '0', '-y', '0', '-z', '0.01', '-Y', '0'],
        output='screen'
    )

    spawn_bot2 = ExecuteProcess(
        cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
             '-topic', '/my_bot2/robot_description',
             '-entity', 'my_bot2',
             '-x', '2', '-y', '0', '-z', '0.01', '-Y', '0'],
        output='screen'
    )

    spawn_bot3 = ExecuteProcess(
        cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
             '-topic', '/my_bot3/robot_description',
             '-entity', 'my_bot3',
             '-x', '4', '-y', '0', '-z', '0.01', '-Y', '0'],
        output='screen'
    )

    # Launch description with arguments for sim time
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        # Add both robot nodes to the launch description
        node_robot_state_publisher_my_bot1,
        node_robot_state_publisher_my_bot2,
        node_robot_state_publisher_my_bot3,

        # Add Gazebo spawns
        spawn_bot1,
        spawn_bot2,
        spawn_bot3,


        initial_cmd_vel,
    ])
