# ============================================================
#                        算个文科生吧
#                   lijinghailjh@163.com
# ============================================================
#  File: ljh_rs_pointcloud_mpu6050.launch.py
#  Description:
#     RealSense D435 点云 + 外部 MPU6050 IMU 一键可视化。
#     - 启动 RealSense 彩色/深度及彩色点云
#     - 自动启用 Madgwick IMU 滤波，发布带姿态的 /imu_filtered
#     - 可选发布 IMU -> 相机的静态 TF
#     - 自动打开 RViz2 显示点云
#
#  Usage:
#     ros2 launch ljh_rtabmap_demo ljh_rs_pointcloud_mpu6050.launch.py
#
#  可选参数：
#     ros2 launch ljh_rtabmap_demo ljh_rs_pointcloud_mpu6050.launch.py \
#          imu_raw_topic:=/imu imu_filtered_topic:=/imu_filtered \
#          imu_to_cam_tx:=0.0 imu_to_cam_ty:=0.02 imu_to_cam_tz:=-0.03
#
#  注意：
#     - 请先确保 mpu6050driver 节点已运行并发布 IMU 数据
#     - 若已有其他 RealSense 节点，请先 pkill -f realsense2_camera_node
# ============================================================

"""Launch RealSense pointcloud with external MPU6050 IMU."""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

# 加载 realsense 官方 launch 模块
sys.path.append(os.path.join(get_package_share_directory('realsense2_camera'), 'launch'))
import rs_launch  # type: ignore  # noqa: E402


# ============================================================
#   参数定义
# ============================================================
local_parameters = [
    {'name': 'camera_name', 'default': 'camera', 'description': 'camera unique name'},
    {'name': 'camera_namespace', 'default': '', 'description': 'camera namespace (empty for simpler topics)'},
    {'name': 'serial_no', 'default': '', 'description': 'RealSense device serial number (optional)'},
    {'name': 'usb_port_id', 'default': '', 'description': 'RealSense USB port id (optional)'},
    {'name': 'enable_color', 'default': 'true', 'description': 'enable color stream'},
    {'name': 'enable_depth', 'default': 'true', 'description': 'enable depth stream'},
    {'name': 'pointcloud.enable', 'default': 'true', 'description': 'enable pointcloud'},
    {'name': 'align_depth.enable', 'default': 'true', 'description': 'align depth to color (required for colored pointcloud)'},
    {'name': 'enable_sync', 'default': 'true', 'description': 'enable frame synchronization'},
    {'name': 'rgb_camera.profile', 'default': '640x360x30', 'description': 'RGB camera profile (widthxheightxfps)'},
    {'name': 'depth_module.profile', 'default': '640x360x30', 'description': 'Depth module profile (widthxheightxfps)'}
]


def declare_argument(name: str, default: str, description: str) -> DeclareLaunchArgument:
    return DeclareLaunchArgument(name, default_value=default, description=description)


def set_configurable_parameters(local_params):
    """将本地参数转换为 LaunchConfiguration 字典"""
    return {param['name']: LaunchConfiguration(param['name']) for param in local_params}


def declare_configurable_parameters(local_params):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in local_params]


def setup_pointcloud_params(context):
    """realsense 节点启动后，循环设置 pointcloud 参数"""
    camera_name = context.launch_configurations.get('camera_name', 'camera')
    camera_namespace = context.launch_configurations.get('camera_namespace', '')

    node_path = f"/{camera_namespace}/{camera_name}" if camera_namespace else f"/{camera_name}"

    enable_cmd = (
        f"for i in {{1..10}}; do "
        f"ros2 param set {node_path} pointcloud__neon_.enable True >/dev/null 2>&1 && break; "
        f"sleep 1; "
        f"done"
    )

    stream_cmd = (
        f"for i in {{1..10}}; do "
        f"ros2 param set {node_path} pointcloud__neon_.stream_filter 2 >/dev/null 2>&1 && break; "
        f"sleep 1; "
        f"done"
    )

    return [
        ExecuteProcess(cmd=['/bin/bash', '-c', enable_cmd], output='screen'),
        ExecuteProcess(cmd=['/bin/bash', '-c', stream_cmd], output='screen')
    ]


def generate_launch_description() -> LaunchDescription:
    # realsense 可配置参数
    rs_params = rs_launch.configurable_parameters
    all_params = local_parameters + rs_params

    # Launch 参数声明
    launch_args = [
        declare_argument('rviz_config', PathJoinSubstitution([
            get_package_share_directory('ljh_rtabmap_demo'), 'rviz', 'pointcloud.rviz'
        ]), 'RViz2 配置文件路径'),
        declare_argument('config_file', PathJoinSubstitution([
            get_package_share_directory('ljh_rtabmap_demo'),
            'config',
            'realsense_empty.json'
        ]), 'RealSense 相机 JSON 配置文件路径（可选）'),
        declare_argument('imu_raw_topic', '/imu', '原始 IMU 话题（mpu6050driver 输出）'),
        declare_argument('imu_filtered_topic', '/imu_filtered', '姿态解算后的 IMU 话题'),
        declare_argument('camera_frame', 'camera_link', 'RealSense 相机坐标系'),
        declare_argument('imu_frame', 'imu_link', 'IMU 坐标系名称'),
        declare_argument('imu_to_cam_tx', '0.0', 'IMU 相对于相机的 X 平移 (m)'),
        declare_argument('imu_to_cam_ty', '0.0', 'IMU 相对于相机的 Y 平移 (m)'),
        declare_argument('imu_to_cam_tz', '0.0', 'IMU 相对于相机的 Z 平移 (m)'),
        declare_argument('imu_to_cam_roll', '0.0', 'IMU 相对于相机的 Roll (rad, about X)'),
        declare_argument('imu_to_cam_pitch', '0.0', 'IMU 相对于相机的 Pitch (rad, about Y)'),
        declare_argument('imu_to_cam_yaw', '0.0', 'IMU 相对于相机的 Yaw (rad, about Z)')
    ]

    # RealSense 官方驱动
    realsense_launch = OpaqueFunction(
        function=rs_launch.launch_setup,
        kwargs={'params': set_configurable_parameters(all_params)}
    )

    # IMU Madgwick 滤波器
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[{
            'use_magnetic_field': False,
            'world_frame': 'enu',
            'publish_tf': False,
        }],
        remappings=[
            ('imu/data_raw', LaunchConfiguration('imu_raw_topic')),
            ('imu/data', LaunchConfiguration('imu_filtered_topic')),
        ],
    )

    # IMU -> 相机静态 TF（可修改平移/姿态）
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_to_camera_tf',
        arguments=[
            LaunchConfiguration('imu_to_cam_tx'),
            LaunchConfiguration('imu_to_cam_ty'),
            LaunchConfiguration('imu_to_cam_tz'),
            LaunchConfiguration('imu_to_cam_roll'),
            LaunchConfiguration('imu_to_cam_pitch'),
            LaunchConfiguration('imu_to_cam_yaw'),
            LaunchConfiguration('imu_frame'),
            LaunchConfiguration('camera_frame'),
        ],
        output='screen',
    )

    # RViz2 可视化
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )

    ld = LaunchDescription()
    for arg in launch_args:
        ld.add_action(arg)

    for decl in declare_configurable_parameters(local_parameters):
        ld.add_action(decl)

    for decl in rs_launch.declare_configurable_parameters(rs_params):
        ld.add_action(decl)

    ld.add_action(realsense_launch)
    ld.add_action(imu_filter_node)
    ld.add_action(TimerAction(period=1.0, actions=[static_tf_node]))
    ld.add_action(TimerAction(period=3.0, actions=[OpaqueFunction(function=setup_pointcloud_params)]))
    ld.add_action(rviz_node)

    return ld


