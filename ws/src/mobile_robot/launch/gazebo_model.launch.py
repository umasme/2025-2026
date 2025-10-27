import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro

def generate_launch_description():

    robotXacroName='differential_drive_robot'

    namePackage = 'mobile_robot'

    # relative path to the xacro file defining the model
    modelFileRelativePath = 'model/robot.xacro'
    # relative path to the Gazebo world file
    worldFileRelativePath = 'model/empty_world.world'

    # absolute path to the world model
    pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)

    # absolute path to the world model
    pathWorldFile = os.path.join(get_package_share_directory(namePackage), worldFileRelativePath)

    # get the robot description from the xacro model file
    robotDescription = xacro.process_file(pathModelFile).toxml()

    # path to Gazebo (gazebo_ros) launch file and its description source
    gazeboLaunchFile = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
    )
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(gazeboLaunchFile)

    # launch description
    gazeboLaunch = IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments = {'world': pathWorldFile}.items())

    # create a gazebo_ros Node
    spawnModelNode = Node(package='gazebo_ros', executable='spawn_entity.py',
                          arguments=['-topic', 'robot_description', '-entity', robotXacroName], output='screen')
    
    # Robot State Publisher Node
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robotDescription, 'use_sim_time': True}]
    )

    # create an empty launch description object
    launchDescriptionObject = LaunchDescription()

    # add gazebo launch
    launchDescriptionObject.add_action(gazeboLaunch)

    # add the two nodes
    launchDescriptionObject.add_action(spawnModelNode)
    launchDescriptionObject.add_action(nodeRobotStatePublisher)

    return launchDescriptionObject