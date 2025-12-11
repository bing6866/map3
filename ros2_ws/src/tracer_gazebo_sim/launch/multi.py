import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    pkg_tracer_description = get_package_share_directory('tracer_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    default_world_path = os.path.join(pkg_tracer_description, 'world', 'custom_room.world')
    xacro_file = os.path.join(pkg_tracer_description, 'urdf', 'tracer_v1.xacro')

    # 1. 启动 Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': default_world_path, 'verbose': 'true'}.items()
    )

    # ========================================================================
    # 定义生成函数
    # ========================================================================
    def spawn_robot(robot_name, x_pos, y_pos):
        # 1. 生成每个机器人独立的 URDF
        # 注意：robot_namespace 参数传递给 xacro，确保插件内部命名空间正确
        robot_description_content = Command([
            'xacro ', xacro_file,
            ' robot_namespace:=', robot_name
        ])
        
        robot_description = {
            'robot_description': ParameterValue(robot_description_content, value_type=str)
        }

        # 2. 设置 frame_prefix，这会加在 base_link 等名字前面
        frame_prefix = robot_name + '/'
        
        return GroupAction([
            # 将该组内所有节点的默认话题前缀设为 robot_name (例如 /robot1/cmd_vel)
            PushRosNamespace(robot_name),
            
            # --- Robot State Publisher (TF 发布者) ---
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                parameters=[
                    robot_description, 
                    {
                        'use_sim_time': True, 
                        'frame_prefix': frame_prefix 
                    }
                ],
                # 【核心修复】：强制将私有的 /robotN/tf 重映射回全局 /tf
                # 这样 Gazebo 插件才能在全局 TF 树中找到这个机器人的 link，从而成功初始化 Scan
                remappings=[
                    ('/tf', '/tf'),
                    ('/tf_static', '/tf_static')
                ]
            ),
            
            # --- Spawn Entity (Gazebo 生成模型) ---
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_entity',
                output='screen',
                arguments=[
                    '-topic', 'robot_description', 
                    '-entity', robot_name,
                    '-x', str(x_pos), '-y', str(y_pos), '-z', '0.1',
                    '-robot_namespace', robot_name
                ]
            ),
        ])

    # ========================================================================
    #  错峰启动逻辑
    # ========================================================================
    
    # Robot 1 (立即启动)
    spawn_robot1 = spawn_robot('robot1', 0.0, 0.0)

    # Robot 2 (延时启动，给 R1 和 Gazebo 初始化留出时间)
    spawn_robot2 = TimerAction(
        period=5.0,
        actions=[spawn_robot('robot2', 2.0, -5.0)]
    )

    # Robot 3
    spawn_robot3 = TimerAction(
        period=10.0,
        actions=[spawn_robot('robot3', -4.0, 3.0)]
    )

    ld = LaunchDescription()
    ld.add_action(gazebo)
    ld.add_action(spawn_robot1)
    ld.add_action(spawn_robot2)
    ld.add_action(spawn_robot3)

    return ld