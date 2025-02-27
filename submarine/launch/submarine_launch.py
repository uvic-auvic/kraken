import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the Gazebo launch file
    gazebo_ros_pkg_path = get_package_share_directory('gazebo_ros')
    gazebo_launch_path = os.path.join(gazebo_ros_pkg_path, 'launch', 'gazebo.launch.py')
    
    # Use the submarine.sdf from the models directory to spawn the model (includes depth_sensor_link)
    submarine_pkg_path = get_package_share_directory('submarine')
    sdf_file = os.path.join(submarine_pkg_path, 'models', 'submarine.sdf')
    
    if not os.path.exists(sdf_file):
        raise FileNotFoundError(f"Could not find submarine SDF at {sdf_file}")
    
    print(f"\n\n***** Using SDF file at: {sdf_file} *****\n\n")
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={'verbose': 'true'}.items()
    )
    
    wait_for_gazebo = ExecuteProcess(
        cmd=['sleep', '10'],
        output='screen'
    )
    
    spawn_entity = Node(
        package='gazebo_ros',
        node_executable='spawn_entity.py',  # using node_executable for ROS2 Eloquent compatibility
        arguments=[
            '-entity', 'simple_submarine', 
            '-file', sdf_file,
            '-x', '0.0', '-y', '0.0', '-z', '1.0',
            '-b'
        ],
        output='screen'
    )
    
    gps_listener = Node(
        package='submarine',
        node_executable='gps_listener.py',  # using node_executable for ROS2 Eloquent compatibility
        name='gps_listener',
        output='screen'
    )

    depth_listener = Node(
        package='submarine',
        node_executable='depth_listener.py',  # using node_executable for ROS2 Eloquent compatibility
        name='depth_listener',
        output='screen'
    )
    
    return LaunchDescription([
        gazebo_launch,
        wait_for_gazebo,
        spawn_entity,
        gps_listener,
        depth_listener
    ])
