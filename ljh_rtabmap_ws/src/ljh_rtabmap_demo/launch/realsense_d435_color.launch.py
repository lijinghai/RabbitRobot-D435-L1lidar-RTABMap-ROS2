# ============================================================
#                        算个文科生吧
#                   lijinghailjh@163.com
# ============================================================
#  File: realsense_d435_color.launch.py
#  Description:
#     启动 Intel RealSense D435 彩色 + 深度 相机，
#     并运行 RTAB-Map SLAM（包含里程计、建图与可视化）。
#
#  Requirements:
#     1. 一台 Intel RealSense D435 相机（非 D435i）
#     2. 已安装 realsense2_camera ROS 2 包：
#        sudo apt install ros-$ROS_DISTRO-realsense2-camera
#     3. 已安装 rtabmap_ros 包：
#        sudo apt install ros-$ROS_DISTRO-rtabmap-ros
#
#  Usage:
#     ros2 launch ljh_rtabmap_demo realsense_d435_color.launch.py
# ============================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    generate_launch_description() 是 ROS 2 启动文件的主入口函数。
    它返回一个 LaunchDescription 对象，包含所有需要同时启动的节点。
    """

    # ============================================================
    #   RTAB-Map 公共参数
    # ============================================================
    parameters = [{
        # 相机的参考坐标系（TF 框架的根节点）
        'frame_id': 'camera_link',

        # 是否订阅深度图像
        'subscribe_depth': True,

        # 是否启用近似时间同步
        # 若 CPU 性能较弱，可设置 True（但建图精度略降）
        'approx_sync': False,
    }]

    # ============================================================
    #    话题重映射（Remappings）
    #    用于对齐不同节点的输入/输出话题名称。
    # ============================================================
    remappings = [
        # 将 RTAB-Map 的 RGB 图像输入映射到 RealSense 彩色图像话题
        ('rgb/image', '/camera/color/image_raw'),

        # 相机内参信息
        ('rgb/camera_info', '/camera/color/camera_info'),

        # 深度图像输入，使用对齐到彩色图的深度图
        ('depth/image', '/camera/aligned_depth_to_color/image_raw')
    ]

    # ============================================================
    #   返回 LaunchDescription 对象，包含所有子节点与参数
    # ============================================================
    return LaunchDescription([

        # ------------------------------------------------------------
        # DeclareLaunchArgument：声明命令行可传入的参数
        #   例如：
        #   ros2 launch ljh_rtabmap_demo realsense_d435_color.launch.py args:="--Vis/MinDepth 0.3"
        # ------------------------------------------------------------
        DeclareLaunchArgument(
            'args', default_value='',
            description='额外参数，用于设置 RTAB-Map 主节点和里程计节点的运行参数。'
        ),

        DeclareLaunchArgument(
            'odom_args', default_value='',
            description='仅传递给里程计节点的额外参数（优先级高于 args）。'
        ),

        # ------------------------------------------------------------
        # IncludeLaunchDescription：
        #   启动 RealSense 官方相机驱动（realsense2_camera）
        # ------------------------------------------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    # 获取 realsense2_camera 包路径
                    get_package_share_directory('realsense2_camera'),
                    'launch',
                    'rs_launch.py'
                )
            ]),
            launch_arguments={
                # 设置命名空间为空，方便后续话题简洁
                'camera_namespace': '',

                # 开启对齐（深度对齐到彩色图像）
                'align_depth.enable': 'true',

                # 启用所有传感器同步采样（保证彩色与深度帧时间一致）
                'enable_sync': 'true',

                # 彩色相机分辨率与帧率
                # 格式为：宽x高x帧率
                'rgb_camera.profile': '640x360x30',

                # 深度模块分辨率与帧率
                'depth_module.profile': '640x360x30',

                # 启用彩色相机
                'enable_color': 'true',

                # 启用深度相机
                'enable_depth': 'true',
            }.items(),
        ),

        # ------------------------------------------------------------
        #    RTAB-Map 视觉里程计节点
        #   功能：估计相机相对运动（位姿）
        # ------------------------------------------------------------
        Node(
            package='rtabmap_odom',         # 所属 ROS 包
            executable='rgbd_odometry',     # 可执行文件名
            output='screen',                # 日志输出到终端
            parameters=parameters,          # 共享参数
            arguments=[LaunchConfiguration("args"),
                       LaunchConfiguration("odom_args")],  # 可通过命令行附加参数
            remappings=remappings           # 话题映射
        ),

        # ------------------------------------------------------------
        #    RTAB-Map SLAM 主节点
        #   功能：执行建图、闭环检测、位置优化等
        # ------------------------------------------------------------
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=parameters,
            remappings=remappings,
            # "-d" 表示启动时删除旧数据库，重新建图
            arguments=['-d', LaunchConfiguration("args")]
        ),

        # ------------------------------------------------------------
        #    RTAB-Map 可视化界面
        #   功能：显示相机轨迹、点云地图、关键帧等
        # ------------------------------------------------------------
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            output='screen',
            parameters=parameters,
            remappings=remappings
        ),
    ])