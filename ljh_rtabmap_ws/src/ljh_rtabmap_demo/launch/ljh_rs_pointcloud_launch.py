# ============================================================
#                        算个文科生吧
#                   lijinghailjh@163.com
# ============================================================
#  File: ljh_rs_pointcloud_launch.py
#  Description:
#     基于 realsense2_camera 官方 rs_pointcloud_launch.py 修改，
#     针对 Intel RealSense D435 优化，自动启用彩色点云。
#     启动后自动打开 RViz2 并显示彩色点云。
#
#  Usage:
#     ros2 launch ljh_rtabmap_demo ljh_rs_pointcloud_launch.py
#
#  可选参数：
#     ros2 launch ljh_rtabmap_demo ljh_rs_pointcloud_launch.py camera_name:=my_camera
#     ros2 launch ljh_rtabmap_demo ljh_rs_pointcloud_launch.py camera_namespace:=my_namespace
#     ros2 launch ljh_rtabmap_demo ljh_rs_pointcloud_launch.py rviz_config:=/path/to/your/config.rviz
#
#  重要提示：
#     - 启动前请确保没有其他 RealSense 节点在运行
#     - 如果出现 "Depth stream start failure" 错误，先执行：
#       pkill -f realsense2_camera_node
#     - 然后重新启动本文件
#
#  功能说明：
#     - 自动启动 RealSense D435 相机
#     - 自动启用深度对齐和同步
#     - 自动启用彩色点云发布
#     - 自动打开 RViz2 并加载点云显示配置
#     - 点云话题：/camera/depth/color/points（默认配置）
# ============================================================

"""Launch realsense2_camera node with pointcloud for D435."""
from launch import LaunchDescription
from launch.actions import OpaqueFunction, TimerAction, ExecuteProcess, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PathJoinSubstitution
from launch_ros.actions import Node
import launch_ros.actions
import sys
import pathlib
import os
from ament_index_python.packages import get_package_share_directory

# 添加 realsense2_camera launch 目录到路径
sys.path.append(os.path.join(get_package_share_directory('realsense2_camera'), 'launch'))
import rs_launch

# 本地参数定义（可被命令行覆盖）
local_parameters = [
    {'name': 'camera_name', 'default': 'camera', 'description': 'camera unique name'},
    {'name': 'camera_namespace', 'default': '', 'description': 'camera namespace (empty for simpler topics)'},
    {'name': 'enable_color', 'default': 'true', 'description': 'enable color stream'},
    {'name': 'enable_depth', 'default': 'true', 'description': 'enable depth stream'},
    {'name': 'pointcloud.enable', 'default': 'true', 'description': 'enable pointcloud'},
    # D435 特定优化参数
    {'name': 'align_depth.enable', 'default': 'true', 'description': 'align depth to color (required for colored pointcloud)'},
    {'name': 'enable_sync', 'default': 'true', 'description': 'enable frame synchronization'},
    {'name': 'rgb_camera.profile', 'default': '640x360x30', 'description': 'RGB camera profile (widthxheightxfps)'},
    {'name': 'depth_module.profile', 'default': '640x360x30', 'description': 'Depth module profile (widthxheightxfps)'},
]

def set_configurable_parameters(local_params):
    """将本地参数转换为 LaunchConfiguration 字典"""
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in local_params])


def setup_pointcloud_params(context):
    """设置点云参数的回调函数"""
    camera_name = context.launch_configurations.get('camera_name', 'camera')
    camera_namespace = context.launch_configurations.get('camera_namespace', '')
    
    # 构建节点完整路径
    # 当 camera_namespace 为空时，节点路径为 /camera_name
    # 当 camera_namespace 不为空时，节点路径为 /camera_namespace/camera_name
    if camera_namespace:
        node_path = f'/{camera_namespace}/{camera_name}'
    else:
        node_path = f'/{camera_name}'
    
    enable_cmd = (
        f"for i in {{1..10}}; do "
        f"ros2 param set {node_path} pointcloud__neon_.enable True >/dev/null 2>&1 && "
        f"break; "
        f"sleep 1; "
        f"done"
    )

    stream_filter_cmd = (
        f"for i in {{1..10}}; do "
        f"ros2 param set {node_path} pointcloud__neon_.stream_filter 2 >/dev/null 2>&1 && "
        f"break; "
        f"sleep 1; "
        f"done"
    )

    return [
        ExecuteProcess(cmd=['/bin/bash', '-c', enable_cmd], output='screen'),
        ExecuteProcess(cmd=['/bin/bash', '-c', stream_filter_cmd], output='screen')
    ]


def generate_launch_description():
    """生成启动描述"""
    # 获取所有可配置参数
    params = rs_launch.configurable_parameters
    
    # 合并本地参数和系统参数（本地参数包含 D435 优化设置）
    all_params = local_parameters + params
    
    # 声明 rviz 配置文件参数（可选）
    # 如果指定了 rviz_config，则使用指定的配置文件
    # 如果未指定，则使用默认路径或让 RViz2 使用默认配置
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            get_package_share_directory('ljh_rtabmap_demo'),
            'rviz',
            'pointcloud.rviz'
        ]),
        description='Path to RViz2 configuration file. If file does not exist, RViz2 will start with default config.'
    )
    
    return LaunchDescription(
        # 声明所有可配置参数
        [rviz_config_arg] +
        rs_launch.declare_configurable_parameters(local_parameters) +
        rs_launch.declare_configurable_parameters(params) +
        [
            # 启动 RealSense 相机节点
            # 传递所有参数（包括本地定义的 D435 优化参数）
            OpaqueFunction(
                function=rs_launch.launch_setup,
                kwargs={'params': set_configurable_parameters(all_params)}
            ),
            
            # 延迟设置点云参数（RealSense 节点启动后 3 秒设置）
            # 点云参数无法在 launch_arguments 中直接设置，需要启动后设置
            # 使用 OpaqueFunction 来动态获取 camera_name 和 camera_namespace
            TimerAction(
                period=3.0,
                actions=[
                    OpaqueFunction(function=setup_pointcloud_params)
                ]
            ),
            
            # 启动 RViz2 用于可视化点云
            # 如果指定了 rviz_config 参数，则加载该配置文件
            launch_ros.actions.Node(
                package='rviz2',
                namespace='',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', LaunchConfiguration('rviz_config')]
            )
        ]
    )

