from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    slam = Node(
        package="slam",
        executable="slam"
    )
    media = Node(
        package="media",
        executable="media"
    )
    model = Node(
        package="model3",
        executable="model3"
    )
    
    launch_description = LaunchDescription([slam,media,model])
    return launch_description