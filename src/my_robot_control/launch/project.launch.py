import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    #turtlebot3 waffle node package location
    turtlebot_pkg = get_package_share_directory('turtlebot3_fake_node')
    
    return LaunchDescription([
        #start the tobot simulationRViz)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot_pkg, 'launch', 'turtlebot3_fake_node.launch.py')
            )
        ),

        #start usbcam drivier
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen'
        ),

        #controller
        Node(
            package='my_robot_control',
            executable='controller',
            name='my_controller',
            output='screen'
        )
    ])
