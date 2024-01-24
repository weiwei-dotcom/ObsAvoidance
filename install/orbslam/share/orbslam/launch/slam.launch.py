from launch import LaunchDescription
from ament_index_python.packages import get_package_share_path
from launch_ros.actions import Node
import os.path
import yaml

def load_param_from(path_param_file):
    with open(path_param_file,'r') as f:
        params = yaml.safe_load(f)
    return params

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
        arguments=['-d', '/home/weiwei/Desktop/project/ObsAvoidance/src/rviz_default.rviz']
    )
    path_planner = Node(
        package="path_planner",
        executable="path_planner",
        parameters=[load_param_from('/home/weiwei/Desktop/project/ObsAvoidance/src/path_planner/path_planner_config.yaml')]
    )
    launch_description = LaunchDescription([usb_cam,slam,media,model,rviz2,path_planner])
    return launch_description