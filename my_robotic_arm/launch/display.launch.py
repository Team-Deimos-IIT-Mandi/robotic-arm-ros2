import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'my_robotic_arm' 
    
    # 1. Finding the path for URDF file
    pkg_share = get_package_share_directory(pkg_name)
    urdf_file = os.path.join(pkg_share, 'urdf', 'Arm_Urdf.urdf')

    # 2. Read the URDF file content
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    robot_desc = robot_desc.replace('${find my_robotic_arm}', pkg_share)

    # 3. Node: Robot State Publisher (Publishes TF for RViz)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    
    )

    # 4. Node: Joint State Publisher GUI 
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # 5. Node: RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    # 6. Include: Gazebo Harmonic (Empty World) 
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 7. Node: Spawn Entity in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_arm',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.05', 
            '-R', '0.0',  # Roll (X-axis rotation) 
            '-P', '0.0',  # Pitch (Y-axis rotation) 
            '-Y', '0.0'   # Yaw (Z-axis rotation)
        ],
        output='screen'
    )
    # Node to start the Joint State Broadcaster (Reads joint angles)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # Node to start the Arm Controller (Moves the robot)
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node,
        gz_sim,
        spawn_entity,
        joint_state_broadcaster_spawner,
        arm_controller_spawner
    ])