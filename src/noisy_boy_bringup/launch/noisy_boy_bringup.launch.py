import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # Retrieve the evaluated LaunchConfiguration values as string
    namespace_str = context.perform_substitution(LaunchConfiguration('namespace'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_path = LaunchConfiguration('world_path')

    pkg_noisy_boy_description = get_package_share_directory('noisy_boy_description')
    pkg_noisy_boy_bringup = get_package_share_directory('noisy_boy_bringup')

    urdf_file_path = os.path.join(pkg_noisy_boy_description, 'urdf', 'noisy_boy.xacro')
    
    # Process xacro with namespace argument
    robot_description_content = Command([
        'xacro ',
        urdf_file_path,
        f' namespace:={namespace_str}'
    ])
    robot_description = {'robot_description': robot_description_content}

    # Node 1: robot_state_publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace_str,
        parameters=[
            robot_description,
            {
                'use_sim_time': use_sim_time,
                'frame_prefix': f"{namespace_str}/" if namespace_str else ""
            }
        ]
    )

    # Node 2: spawn_noisy_boy
    spawn_noisy_boy = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_noisy_boy",
        namespace=namespace_str,
        output="screen",
        arguments=[
            '-topic', 'robot_description',
            '-name', namespace_str,
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Node 3: gazebo_bridge (parameter_bridge)
    # We construct parameter bridge arguments dynamically!
    bridge_args = [
        '/world/empty/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/world/complex_maze_world/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        f'/model/{namespace_str}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        f'/model/{namespace_str}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        f'/model/{namespace_str}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        f'/model/{namespace_str}/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        f'/model/{namespace_str}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        f'/model/{namespace_str}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
    ]

    bridge_remappings = [
        (f'/model/{namespace_str}/tf', '/tf'),
        (f'/model/{namespace_str}/cmd_vel', f'/{namespace_str}/cmd_vel'),
        (f'/model/{namespace_str}/odom', f'/{namespace_str}/odom'),
        (f'/model/{namespace_str}/ground_truth', f'/{namespace_str}/ground_truth'),
        (f'/model/{namespace_str}/joint_states', f'/{namespace_str}/joint_states'),
        (f'/model/{namespace_str}/scan', f'/{namespace_str}/scan'),
    ]

    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="parameter_bridge",
        output="screen",
        arguments=bridge_args,
        remappings=bridge_remappings,
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Node 4: RViz with dynamic configuration
    rviz_template_path = os.path.join(pkg_noisy_boy_bringup, 'rviz', 'noisy_boy.rviz')
    temp_rviz_dir = os.path.join(pkg_noisy_boy_bringup, 'rviz', 'tmp')
    os.makedirs(temp_rviz_dir, exist_ok=True)
    temp_rviz_path = os.path.join(temp_rviz_dir, f'noisy_boy_{namespace_str}.rviz')

    with open(rviz_template_path, 'r') as f:
        rviz_content = f.read()

    # Perform the replacements:
    # 1. Fixed Frame: odom -> Fixed Frame: {namespace_str}/odom
    rviz_content = rviz_content.replace('Fixed Frame: odom', f'Fixed Frame: {namespace_str}/odom')
    # 2. Value: /robot_description -> Value: /{namespace_str}/robot_description
    rviz_content = rviz_content.replace('Value: /robot_description', f'Value: /{namespace_str}/robot_description')
    # 3. Value: /scan -> Value: /{namespace_str}/scan
    rviz_content = rviz_content.replace('Value: /scan', f'Value: /{namespace_str}/scan')
    # 4. TF Prefix: "" -> TF Prefix: {namespace_str}
    rviz_content = rviz_content.replace('TF Prefix: ""', f'TF Prefix: "{namespace_str}"')

    with open(temp_rviz_path, 'w') as f:
        f.write(rviz_content)

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', temp_rviz_path],
    )

    spawn_robot_str = context.perform_substitution(LaunchConfiguration('spawn_robot'))
    if spawn_robot_str.lower() == 'true':
        return [
            robot_state_publisher,
            spawn_noisy_boy,
            gazebo_bridge,
            rviz_node
        ]
    else:
        # Multi-robot spawner mode: we want to launch RViz with 'map' as the fixed frame!
        map_rviz_path = os.path.join(temp_rviz_dir, 'noisy_boy_map.rviz')
        
        with open(rviz_template_path, 'r') as f:
            rviz_content = f.read()
            
        # Set Fixed Frame to map
        rviz_content = rviz_content.replace('Fixed Frame: odom', 'Fixed Frame: map')
        # Pre-configure to show robot1 by default
        rviz_content = rviz_content.replace('Value: /robot_description', 'Value: /robot1/robot_description')
        rviz_content = rviz_content.replace('Value: /scan', 'Value: /robot1/scan')
        rviz_content = rviz_content.replace('TF Prefix: ""', 'TF Prefix: "robot1"')
        
        with open(map_rviz_path, 'w') as f:
            f.write(rviz_content)
            
        map_rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', map_rviz_path],
        )
        return [map_rviz_node]

def generate_launch_description():
    pkg_noisy_boy_bringup = get_package_share_directory('noisy_boy_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_world_path_cmd = DeclareLaunchArgument(
        "world_path",
        default_value="empty.sdf",
        description="Name of the world file to load (empty.sdf, world1.sdf, world2.sdf)"
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="robot1",
        description="Top-level namespace for the robot, TF frames, and topics"
    )

    declare_spawn_robot_cmd = DeclareLaunchArgument(
        "spawn_robot",
        default_value="true",
        description="Whether to spawn the default robot (set to false if using the spawner node)"
    )

    world_path = LaunchConfiguration("world_path")
    world_file_path = PathJoinSubstitution(
        [
            pkg_noisy_boy_bringup,
            "worlds",
            world_path
        ]
    )

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -v 4 ', world_file_path]
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_world_path_cmd,
        declare_namespace_cmd,
        declare_spawn_robot_cmd,
        gazebo_sim,
        OpaqueFunction(function=launch_setup)
    ])