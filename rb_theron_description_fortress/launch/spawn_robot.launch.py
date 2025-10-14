import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Explicitly set the Gazebo resource path
    pkg_share = get_package_share_directory('rb_theron_description_fortress')
    resource_path = os.path.join(pkg_share, '..') # We need the parent of the package's share directory

    set_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=resource_path
    )

    # Correct path to the world file
    world_path = '~/warehouse_ws/src/warehouse-simulation/final_world/industrial-warehouse.sdf'

    # Path to the robot's URDF file
    robot_description_path = os.path.join(
        get_package_share_directory('rb_theron_description_fortress'),
        'robots',
        'rb_theron.urdf.xacro'
    )

    # Process the URDF file
    doc = xacro.parse(open(robot_description_path))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}
    
    # Path to the controller config file
    controller_config = os.path.join(
        get_package_share_directory('rb_theron_description_fortress'),
        'config',
        'controllers.yaml'
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    # ros2_control node
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config],
        output='screen',
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Spawn entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-entity', 'rb_theron'],
        output='screen'
    )
    
    # Spawn diff_drive_controller
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    )

    # Spawn joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        set_resource_path,
        gazebo,
        control_node,
        robot_state_publisher,
        spawn_entity,
        diff_drive_controller_spawner,
        joint_state_broadcaster_spawner
    ])