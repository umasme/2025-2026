from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from xacro import process_file
import os

def generate_launch_description():
    pkg = get_package_share_directory('mobile_robot')
    world = os.path.join(pkg, 'worlds', 'empty.world')
    xacro_path = os.path.join(pkg, 'urdf', 'four_wheel_rover.urdf.xacro')

    # Process the Xacro into plain URDF
    urdf_xml = process_file(xacro_path).toxml()
    urdf_file = os.path.join(pkg, 'urdf', 'four_wheel_rover.urdf')
    with open(urdf_file, 'w') as f:
        f.write(urdf_xml)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]
        ),
        launch_arguments={'world': world}.items()
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'rover', '-file', urdf_file],
        output='screen'
    )

    return LaunchDescription([gazebo, spawn])
