# ObsAvoidance
This branch is used for trying to code the path planning method, this is the derive branch of PreModelScheme
this branch is used for trying to code the path planning method, this is the derive branch of PreModelScheme
# 系统工作流程讲解：
#   提前标定：基座行程的起始点以及末端点在运动捕捉系统下的三维位置坐标，障碍物前板子结构件在运动捕捉系统坐标系下的位姿，相机到末端工具坐标系之间的位姿转换矩阵。
#   启动系统，逐个关节固定拉升绳索将机械臂固定，motion_control 节点通过与板子通信获取此时电机的编码器位置确定为初始位姿下的电机状态;
#   motion_control 与电机之间进行通信控制机械臂做动作，在动作开始前通过运动捕捉系统记录此时末端工具坐标系的位姿到变量 tool_pose_before_move下，orbslam 节点传来的末端位姿始终记录在一个current_pose 下，记录动作开始前的current_pose 变量到 cam_pose_before_move 下；等待动作完毕后同样通过运动捕捉系统
# 系统分工：
#   节点：
#   path_planner: 
#       First, recieve the point_cloud of obstaccle
#   path_follow: 
#   orbslam:   
#   model: 

