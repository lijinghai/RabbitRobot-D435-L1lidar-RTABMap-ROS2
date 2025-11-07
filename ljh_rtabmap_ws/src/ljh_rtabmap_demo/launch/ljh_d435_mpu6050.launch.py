# ============================================================
#                        算个文科生吧
#                   lijinghailjh@163.com
# ============================================================
#  File: ljh_d435_mpu6050.launch.py
#  Description:
#     一键启动 Intel RealSense D435 + MPU6050 IMU 与 RTAB-Map SLAM。
#     - 启动 RealSense 官方驱动（彩色 + 深度，对齐 & 同步）
#     - 订阅外部 IMU（默认话题 /imu）并接入 RTAB-Map
#     - 启动 rgbd_odometry、rtabmap、rtabmap_viz
#     - 可选发布 IMU → 相机 的静态 TF（默认零位姿，可按需修改）
#
#  Usage:
#     ros2 launch ljh_rtabmap_demo ljh_d435_mpu6050.launch.py
#
#  常用可选参数：
#     ros2 launch ljh_rtabmap_demo ljh_d435_mpu6050.launch.py imu_topic:=/imu_filtered
#     ros2 launch ljh_rtabmap_demo ljh_d435_mpu6050.launch.py imu_to_cam_xyz:="0 0 0" imu_to_cam_rpy:="0 0 0"
#
#  前置依赖：
#     - RealSense D435 相机硬件 & librealsense / realsense2_camera
#     - MPU6050 驱动节点（需先行启动，或在其他工作区运行）
#     - rtabmap_ros（rtabmap_odom、rtabmap_slam、rtabmap_viz）
# ============================================================

import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def declare_argument(name: str, default_value: str, description: str) -> DeclareLaunchArgument:
    return DeclareLaunchArgument(name, default_value=default_value, description=description)


def generate_launch_description() -> LaunchDescription:
    # ------------------------------------------------------------
    # Launch 参数声明
    # ------------------------------------------------------------
    launch_args: List[DeclareLaunchArgument] = [
        declare_argument('args', '', '传递给 RTAB-Map 主节点与里程计节点的额外参数。'),
        declare_argument('odom_args', '', '仅传递给里程计节点的额外参数（优先级高于 args）。'),
        declare_argument('imu_raw_topic', '/imu', '原始 IMU 话题（来自 MPU6050 驱动）。'),
        declare_argument('imu_filtered_topic', '/imu_filtered', '姿态解算后的 IMU 话题。'),
        declare_argument('camera_frame', 'camera_link', 'RealSense 主相机坐标系。'),
        declare_argument('imu_frame', 'imu_link', 'MPU6050 IMU 坐标系名称。'),
        declare_argument('imu_to_cam_tx', '0.0', 'IMU 相对于相机的 X 平移（米）。'),
        declare_argument('imu_to_cam_ty', '0.0', 'IMU 相对于相机的 Y 平移（米）。'),
        declare_argument('imu_to_cam_tz', '0.0', 'IMU 相对于相机的 Z 平移（米）。'),
        declare_argument('imu_to_cam_roll', '0.0', 'IMU 相对于相机的 Roll（绕 X，弧度）。'),
        declare_argument('imu_to_cam_pitch', '0.0', 'IMU 相对于相机的 Pitch（绕 Y，弧度）。'),
        declare_argument('imu_to_cam_yaw', '0.0', 'IMU 相对于相机的 Yaw（绕 Z，弧度）。'),
    ]

    # ------------------------------------------------------------
    # RTAB-Map 公共参数（供 rgbd_odometry / rtabmap / rtabmap_viz 共用）
    # ------------------------------------------------------------
    parameters = [{
        'frame_id': LaunchConfiguration('camera_frame'),
        'subscribe_depth': True,
        'approx_sync': False,
        'always_process_most_recent_frame': False,
        'subscribe_imu': True,
    }]

    # ------------------------------------------------------------
    # 话题重映射配置
    # ------------------------------------------------------------
    imu_filtered_topic = LaunchConfiguration('imu_filtered_topic')
    remappings = [
        ('rgb/image', '/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
        ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
        ('imu', imu_filtered_topic),
    ]

    # ------------------------------------------------------------
    # RealSense 官方驱动（rs_launch.py）
    # ------------------------------------------------------------
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ]),
        launch_arguments={
            'camera_namespace': '',
            'align_depth.enable': 'true',
            'enable_sync': 'true',
            'enable_color': 'true',
            'enable_depth': 'true',
            'rgb_camera.profile': '640x360x30',
            'depth_module.profile': '640x360x30',
        }.items(),
    )

    # ------------------------------------------------------------
    # IMU Madgwick 滤波器：将原始加速度/角速度解算成姿态
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
            ('imu/data', imu_filtered_topic),
        ],
    )

    # 可选：IMU → 相机静态变换（默认 0，可按需调参）
    # ------------------------------------------------------------
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_to_camera_tf',
        arguments=[
            LaunchConfiguration('imu_to_cam_tx'),
            LaunchConfiguration('imu_to_cam_ty'),
            LaunchConfiguration('imu_to_cam_tz'),
            LaunchConfiguration('imu_to_cam_yaw'),
            LaunchConfiguration('imu_to_cam_pitch'),
            LaunchConfiguration('imu_to_cam_roll'),
            LaunchConfiguration('imu_frame'),
            LaunchConfiguration('camera_frame'),
        ],
        output='screen',
    )

    # ------------------------------------------------------------
    # RTAB-Map 各节点
    # ------------------------------------------------------------
    rgbd_odometry_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=parameters,
        arguments=[LaunchConfiguration('args'), LaunchConfiguration('odom_args')],
        remappings=remappings,
    )

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=parameters,
        remappings=remappings,
        arguments=['-d', LaunchConfiguration('args')],
    )

    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=parameters,
        remappings=remappings,
    )

    # ------------------------------------------------------------
    # 组装 LaunchDescription
    # ------------------------------------------------------------
    ld = LaunchDescription()
    for arg in launch_args:
        ld.add_action(arg)

    ld.add_action(realsense_launch)
    ld.add_action(imu_filter_node)

    # 延迟发布静态 TF，确保参数解析完成
    ld.add_action(TimerAction(period=1.0, actions=[static_tf_node]))

    ld.add_action(rgbd_odometry_node)
    ld.add_action(rtabmap_node)
    ld.add_action(rtabmap_viz_node)

    return ld


