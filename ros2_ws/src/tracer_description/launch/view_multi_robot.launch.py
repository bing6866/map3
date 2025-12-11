import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_tracer_description = get_package_share_directory('tracer_description')
    
    # 指向你的 RViz 配置文件
    # 建议你先保存一个新的配置文件，比如 multi_bot.rviz
    default_rviz_config_path = os.path.join(pkg_tracer_description, 'rviz', 'multi_bot.rviz')

    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig', 
        default_value=str(default_rviz_config_path),
        description='Absolute path to rviz config file'
    )

    # 只需要启动 RViz2，并且开启 use_sim_time
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': True}] 
    )

    return LaunchDescription([
        rviz_arg,
        rviz_node
    ])