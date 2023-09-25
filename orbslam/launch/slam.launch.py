from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
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
    
    launch_description = LaunchDescription([slam,media,model])
    return launch_description