import os
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    ld = LaunchDescription()

    # ========================================================================
    # 定义启动单个 Gmapping 的函数
    # ========================================================================
    def create_gmapping_group(robot_name):
        return GroupAction([
            # 1. 放入命名空间 (例如 /robot1)
            PushRosNamespace(robot_name),

            # 2. 启动 slam_gmapping
            Node(
                package='slam_gmapping',
                executable='slam_gmapping',
                name='slam_gmapping',
                output='screen',
                parameters=[{
                    'use_sim_time': True, # 仿真必须开启
                    
                    # === 关键配置 1: 坐标系名称 ===
                    # 必须加上 robot_name 前缀，否则三棵 TF 树会打架
                    'base_frame': f'{robot_name}/base_link',
                    'odom_frame': f'{robot_name}/odom',
                    'map_frame':  f'{robot_name}/map',
                    
                    # === 关键配置 2: 扫描话题 ===
                    # Gmapping 默认订阅 "scan"，因为我们在命名空间下，
                    # 它会自动变成 "/robotX/scan"，所以这里通常不需要改，
                    # 但为了保险，明确指定一下
                    'scan_topic': 'scan',

                    # === 其他 Gmapping 常用参数 (可选调整) ===
                    'map_update_interval': 1.0,
                    'maxUrange': 10.0,  # 激光雷达最大可用距离
                    'sigma': 0.05,
                    'kernelSize': 1,
                    'lstep': 0.05,
                    'astep': 0.05,
                    'iterations': 5,
                    'lsigma': 0.075,
                    'ogain': 3.0,
                    'lskip': 0,
                    'srr': 0.1,
                    'srt': 0.2,
                    'str': 0.1,
                    'stt': 0.2,
                    'linearUpdate': 0.2, # 机器人移动多少米更新一次地图
                    'angularUpdate': 0.2, # 机器人旋转多少弧度更新一次地图
                    'temporalUpdate': 3.0,
                    'resampleThreshold': 0.5,
                    'particles': 30,      # 粒子数，越多越准但越卡
                    'xmin': -20.0,        # 地图初始大小
                    'ymin': -20.0,
                    'xmax': 20.0,
                    'ymax': 20.0,
                    'delta': 0.05,        # 地图分辨率 (米/像素)
                    'llsamplerange': 0.01,
                    'llsamplestep': 0.01,
                    'lasamplerange': 0.005,
                    'lasamplestep': 0.005,
                }],
                # Gmapping 默认发布 'map' 话题，在命名空间下会自动变成 '/robotX/map'
                # 默认发布 'map_metadata'，会自动变成 '/robotX/map_metadata'
                # 所以通常不需要 remappings，除非你的话题名很特殊
                remappings=[
                    # 如果你的雷达话题叫 /robot1/rslidar/scan，这里就需要重映射
                    # ('scan', 'rslidar/scan') 
                ]
            )
        ])

    # ========================================================================
    # 启动三个实例
    # ========================================================================
    ld.add_action(create_gmapping_group('robot1'))
    ld.add_action(create_gmapping_group('robot2'))
    ld.add_action(create_gmapping_group('robot3'))

    return ld