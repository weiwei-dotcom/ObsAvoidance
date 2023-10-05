from launch import LaunchDescription
from launch_ros.actions import Node

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
    
    launch_description = LaunchDescription([usb_cam,slam,media,model])
    return launch_description