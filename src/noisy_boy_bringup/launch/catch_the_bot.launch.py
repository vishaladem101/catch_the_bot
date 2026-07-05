import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_noisy_boy_bringup = get_package_share_directory('noisy_boy_bringup')
    
    # Expose world_name argument (empty, world1, world2)
    declare_world_name_cmd = DeclareLaunchArgument(
        'world_name',
        default_value='empty',
        description='Name of the Gazebo world (empty, world1, world2)'
    )
    
    world_name = LaunchConfiguration('world_name')
    
    # Construct world_path.sdf dynamically
    world_path = [world_name, '.sdf']
    
    # 1. Include base launch with spawn_robot set to false (Gazebo + RViz only)
    base_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_noisy_boy_bringup, 'launch', 'noisy_boy_bringup.launch.py')
        ),
        launch_arguments={
            'spawn_robot': 'false',
            'world_path': world_path
        }.items()
    )

    # 2. Clock bridge node (bridges simulation clock to ROS 2 /clock topic)
    clock_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=[
            ['/world/', world_name, '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
        ],
        remappings=[
            (['/world/', world_name, '/clock'], '/clock')
        ]
    )

    # 3. Spawner node (from noisy_boy_spawner package)
    spawner_node = Node(
        package='noisy_boy_spawner',
        executable='spawner',
        name='noisy_spawner',
        output='screen',
        parameters=[{
            'world_name': world_name,
            'use_sim_time': True
        }]
    )

    # 4. Controller node (from noisy_boy_controller package)
    controller_node = Node(
        package='noisy_boy_controller',
        executable='controller',
        name='noisy_controller',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        declare_world_name_cmd,
        base_bringup,
        clock_bridge_node,
        spawner_node,
        controller_node
    ])
