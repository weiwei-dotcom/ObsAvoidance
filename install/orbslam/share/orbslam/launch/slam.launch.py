from launch import LaunchDescription
from ament_index_python.packages import get_package_share_path
from launch_ros.actions import Node
import os.path

def generate_launch_description():
    
    usb_cam = Node(
        package = "usb_cam",
        executable="usb_cam_node_exe"
    )
    slam = Node(
        package="orbslam",
        executable="orbslam"
    )
    media = Node(
        package="media",
        executable="media"
    )
    model = Node(
        package="model",
        executable="model"
    )
    rviz2 = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', '/home/weiwei/Desktop/project/ObsAvoidance/src/rviz2_default.rviz']
    )
    
    launch_description = LaunchDescription([usb_cam,slam,media,model,rviz2])
    return launch_description