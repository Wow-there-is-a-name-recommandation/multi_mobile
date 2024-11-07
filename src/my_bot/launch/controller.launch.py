import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    # Simulation time argument
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time if true')
    
    # Use LaunchConfiguration to get the declared launch argument
    use_sim_time_config = LaunchConfiguration('use_sim_time')

    # Node to launch consensus_controller.py for robot1, robot2, robot3
    robot1_consensus_controller = Node(
        package='mobile_controller',
        executable='consensus_controller',
        name='consensus_controller_my_bot1',
        output='both',
        arguments=['my_bot1', '[4.0, 5.0, 6.0]'],
        parameters=[{'use_sim_time': use_sim_time_config}],
    )

    robot2_consensus_controller = Node(
        package='mobile_controller',
        executable='consensus_controller',
        name='consensus_controller_my_bot2',
        output='both',
        arguments=['my_bot2', '[8.0, 4.0, 2.0]'],
        parameters=[{'use_sim_time': use_sim_time_config}],
    )

    robot3_consensus_controller = Node(
        package='mobile_controller',
        executable='consensus_controller',
        name='consensus_controller_my_bot3',
        output='both',
        arguments=['my_bot3', '[3.0, 6.0, 4.0]'],
        parameters=[{'use_sim_time': use_sim_time_config}],
    )

    
    # Node to launch state_communicate.py for robot1, robot2, robot3
    robot1_state_communicate = Node(
        package='mobile_controller',
        executable='state_communicate',
        name='state_communicate_my_bot1',
        output='screen',
        arguments=['my_bot1'],
        parameters=[{'use_sim_time': use_sim_time_config}],
    )

    robot2_state_communicate = Node(
        package='mobile_controller',
        executable='state_communicate',
        name='state_communicate_my_bot2',
        output='screen',
        arguments=['my_bot2'],
        parameters=[{'use_sim_time': use_sim_time_config}],
    )

    robot3_state_communicate = Node(
        package='mobile_controller',
        executable='state_communicate',
        name='state_communicate_my_bot3',
        output='screen',
        arguments=['my_bot3'],
        parameters=[{'use_sim_time': use_sim_time_config}],
    )

    
    # Laplacian algorithm
    robot1_laplacian_algorithm = Node(
        package='mobile_controller',
        executable='laplacian_algorithm',
        name='laplacian_algorithm_my_bot1',
        output='both',
        arguments=['my_bot1', '[4.0, 5.0, 6.0]'],
        parameters=[{'use_sim_time': use_sim_time_config}],
    )

    robot2_laplacian_algorithm = Node(
        package='mobile_controller',
        executable='laplacian_algorithm',
        name='laplacian_algorithm_my_bot2',
        output='both',
        arguments=['my_bot2', '[8.0, 4.0, 2.0]'],
        parameters=[{'use_sim_time': use_sim_time_config}],
    )

    robot3_laplacian_algorithm = Node(
        package='mobile_controller',
        executable='laplacian_algorithm',
        name='laplacian_algorithm_my_bot3',
        output='both',
        arguments=['my_bot3', '[3.0, 6.0, 4.0]'],
        parameters=[{'use_sim_time': use_sim_time_config}],
    )


    # Launch description to add all nodes
    return LaunchDescription([
        use_sim_time,

        robot1_state_communicate,
        robot2_state_communicate,
        robot3_state_communicate,

        robot1_laplacian_algorithm,
        robot2_laplacian_algorithm,
        robot3_laplacian_algorithm,

        robot1_consensus_controller,
        robot2_consensus_controller,
        robot3_consensus_controller,

    ])
