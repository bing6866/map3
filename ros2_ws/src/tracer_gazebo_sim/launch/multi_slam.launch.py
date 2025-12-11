import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    # ========================================================================
    # 设置参数文件路径
    # 请修改 'your_package_name' 为你实际存放 yaml 文件的包名
    # ========================================================================
    # 如果你没有专门的包，可以暂时写死绝对路径，或者使用 slam_toolbox 默认的
    # config_file = '/home/你的用户名/ros2_ws/src/你的包/config/mapper_params_online_async.yaml'
    
    # 这里演示使用 slam_toolbox 自带的默认配置 (如果没有自定义的话)
    slam_pkg_path = get_package_share_directory('slam_toolbox')
    config_file = os.path.join(slam_pkg_path, 'config', 'mapper_params_online_async.yaml')

    ld = LaunchDescription()

    # ========================================================================
    # 定义启动单个 SLAM 的函数
    # ========================================================================
    def create_slam_group(robot_name):
        return GroupAction([
            # 1. 将该组内的所有节点放入 robot_name 命名空间 (例如 /robot1)
            PushRosNamespace(robot_name),

            # 2. 启动 SLAM Toolbox
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    config_file,  # 加载基础配置
                    {
                        'use_sim_time': True, # 仿真必须开启
                        
                        # === 关键：动态覆盖 Frame ID ===
                        # 这取决于你的 xacro/urdf 是否给 frame 加了前缀
                        # 如果你的 TF 树是 robot1/odom -> robot1/base_link
                        'base_frame': f'{robot_name}/base_link',
                        'odom_frame': f'{robot_name}/odom',
                        'map_frame':  f'{robot_name}/map', 
                        
                        # === 关键：话题名称 ===
                        # 因为我们在 Namespace 下，'scan' 会自动解析为 '/robotX/scan'
                        'scan_topic': 'scan', 
                    }
                ],
                # 确保输出的 map 话题也是相对路径
                # 默认 slam_toolbox 发布 'map'，在 namespace 下变为 '/robotX/map'
                remappings=[
                    ('/map', 'map'), 
                    ('/map_metadata', 'map_metadata')
                ]
            )
        ])

    # ========================================================================
    # 启动三个实例
    # ========================================================================
    ld.add_action(create_slam_group('robot1'))
    ld.add_action(create_slam_group('robot2'))
    ld.add_action(create_slam_group('robot3'))

    return ld