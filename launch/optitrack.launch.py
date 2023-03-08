from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    log_level = "info"
    ld = LaunchDescription()
    #Create the NatNet client node
    config = os.path.join(get_package_share_directory('mocap_optitrack_client'),'config','natnetclient.yaml')
    natnet_client = Node(package='mocap_optitrack_client',
                         executable='mocap_optitrack_client',
                         name='natnet_client',
                         parameters = [config],
                         arguments=['--ros-args', '--log-level', log_level])
    #Add the node to the launch description and return it
    ld.add_action(natnet_client)
    return ld