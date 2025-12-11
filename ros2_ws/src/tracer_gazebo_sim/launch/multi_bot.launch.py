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
        # 注意：确保你的 xacro 文件能正确处理 robot_namespace 参数
        # 从而给 frame_id 加上前缀 (例如 robot1/base_link, robot1/odom)
        # 否则三棵树的 frame 名字如果都叫 base_link 会在 Rviz 中混淆
        robot_description_content = Command([
            'xacro ', xacro_file,
            ' robot_namespace:=', robot_name
        ])
        
        robot_description = {
            'robot_description': ParameterValue(robot_description_content, value_type=str)
        }

        return GroupAction([
            PushRosNamespace(robot_name),
            
            # Robot State Publisher 
            # 负责发布 base_link -> wheel_link 等自身的静态变换
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                parameters=[
                    robot_description, 
                    {
                        'use_sim_time': True,
                        # 如果你的 xacro 没有自动加前缀，可以在这里尝试设置 frame_prefix
                        # 'frame_prefix': robot_name + '/' 
                    }
                ]
            ),
            
            # Spawn Entity
            # 将模型加载到 Gazebo 中
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
    #  启动逻辑
    # ========================================================================
    
    # Robot 1 在 (0, 0)
    spawn_robot1 = spawn_robot('robot1', 0.0, 0.0)

    # Robot 2 在 (2.0, -5.0) - 延时启动避免 Gazebo 资源竞争
    spawn_robot2 = TimerAction(
        period=1.0,
        actions=[spawn_robot('robot2', 2.0, -5.0)]
    )

    # Robot 3 在 (-4.0, 3.0)
    spawn_robot3 = TimerAction(
        period=2.0,
        actions=[spawn_robot('robot3', -4.0, 3.0)]
    )

    ld = LaunchDescription()
    
    # 启动 Gazebo 环境
    ld.add_action(gazebo)
    
    # 仅启动机器人生成 (Spawn) 和 状态发布 (Robot State Publisher)
    # 不再发布 world -> map 的 TF
    ld.add_action(spawn_robot1)
    ld.add_action(spawn_robot2)
    ld.add_action(spawn_robot3)

    return ld