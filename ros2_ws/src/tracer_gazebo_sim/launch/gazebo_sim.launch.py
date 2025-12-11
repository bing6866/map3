import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    robot_name = 'tracer'
    urdf_tutorial_path = get_package_share_directory('tracer_description')
    # 可选：自定义世界文件（确保路径正确，否则注释掉用默认空世界）
    default_world_path = urdf_tutorial_path + '/world/custom_room.world'
    default_model_path = urdf_tutorial_path + '/urdf/tracer_v1.xacro'

    # 为 Launch 声明参数
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_model_path),
        description='URDF 的绝对路径')

    # 生成机器人描述（xacro解析为urdf）
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ', launch.substitutions.LaunchConfiguration('model')]
        ),
        value_type=str
    )

    # 启动robot_state_publisher
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    # 启动Gazebo（取消注释，并确保路径正确）
    gazebo_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'), 
            '/launch/gazebo.launch.py'
        ]),
        # 若使用自定义世界，取消下面一行注释；否则删除该参数
        launch_arguments=[('world', default_world_path),('verbose','true')]
    )

    # 在Gazebo中加载机器人模型
    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', robot_name],
    )

    # 启动顺序：先启动Gazebo，再启动模型加载（关键！）
    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        gazebo_launch,  # 先启动Gazebo
        robot_state_publisher_node,  # 发布机器人描述
        spawn_entity_node,  # 最后加载模型
    ])
