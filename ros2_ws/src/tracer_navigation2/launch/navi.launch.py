import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 1. 获取目录路径
    tracer_navigation2_dir = get_package_share_directory('tracer_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    
    # 2. 定义 Launch 配置变量
    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='true')
    
    # 新增：定义 slam 变量，默认为 True
    slam = launch.substitutions.LaunchConfiguration('slam', default='True')

    # 注意：虽然我们不加载地图文件，但 params_file 依然需要，
    # 用于配置代价地图(costmap)、规划器(planner)和控制器(controller)
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file', default=os.path.join(tracer_navigation2_dir, 'config', 'nav2_params.yaml'))

    return launch.LaunchDescription([
        # 3. 声明 Launch 参数
        launch.actions.DeclareLaunchArgument(
            'use_sim_time', 
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        # 新增：声明 slam 参数，允许在命令行覆盖 (ros2 launch ... slam:=False)
        launch.actions.DeclareLaunchArgument(
            'slam', 
            default_value=slam,
            description='Whether run a SLAM'),

        launch.actions.DeclareLaunchArgument(
            'params_file', 
            default_value=nav2_param_path,
            description='Full path to param file to load'),

        # 4. 包含 Nav2 Bringup
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            # 使用 Launch 参数替换原有参数
            launch_arguments={
                'slam': slam,             # 关键点：设置为 True 启用建图
                'map': '',                # 关键点：建图模式下不需要加载现有的 yaml 地图，留空即可
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        ),

        # 5. 启动 RViz2
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])