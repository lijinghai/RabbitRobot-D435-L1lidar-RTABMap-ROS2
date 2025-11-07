import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    localization = LaunchConfiguration('localization')
    rviz = LaunchConfiguration('rviz')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')

    # RTAB-Map 参数
    parameters_slam = {
        'frame_id': 'base_link',  # 使用 base_link 作为根坐标系
        'odom_frame_id': 'odom',
        'map_frame_id': 'map',
        'subscribe_rgbd': True,
        'subscribe_scan_cloud': True,
        'subscribe_scan': False,
        'approx_sync': True,
        'approx_sync_max_interval': 0.5,  # 增加时间同步容忍度
        'sync_queue_size': 200,
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/ProximityByTime': 'false',
        'RGBD/ProximityPathMaxNeighbors': '10',
        'Reg/Strategy': '1',
        'Vis/MinInliers': '12',
        'RGBD/OptimizeFromGraphEnd': 'false',
        'RGBD/OptimizeMaxError': '4',
        'Reg/Force3DoF': 'false',
        'Grid/FromDepth': 'true',
        'Mem/STMSize': '30',
        'RGBD/LocalRadius': '10',
        'Icp/CorrespondenceRatio': '0.2',
        'Icp/PM': 'false',
        'Icp/PointToPlane': 'true',
        'Icp/Iterations': '30',
        'Icp/MaxCorrespondenceDistance': '0.3',
        'Icp/VoxelSize': '0.1',
        'qos_image': 2,
        'qos_scan': 2,
        'topic_queue_size': 200
    }

    # ICP 里程计参数
    parameters_odom = {
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'publish_tf': True,
        'approx_sync': True,
        'queue_size': 200,
        'Icp/PointToPlane': 'true',
        'Icp/Iterations': '30',
        'Icp/VoxelSize': '0.1',
        'Icp/MaxCorrespondenceDistance': '0.3',
        'Icp/CorrespondenceRatio': '0.2',
        'Odom/Strategy': '1',
        'Odom/GuessMotion': 'true',
        'OdomF2M/ScanKeyFrameThr': '0.1'
    }

    # 话题重映射
    remappings = [
        ('rgb/image', '/camera/color/image_raw'),
        ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
        ('rgb/camera_info', '/camera/aligned_depth_to_color/camera_info'),
        ('scan_cloud', '/unilidar/cloud_throttled')
    ]

    # RViz 配置文件
    config_rviz = os.path.join(
        get_package_share_directory('rtabmap_demos'), 'config', 'demo_robot_mapping.rviz'
    )

    return LaunchDescription([

        # 声明启动参数
        DeclareLaunchArgument('rtabmap_viz', default_value='true', description='Launch RTAB-Map UI (optional).'),
        DeclareLaunchArgument('rviz', default_value='true', description='Launch RVIZ (optional).'),
        DeclareLaunchArgument('localization', default_value='false', description='Launch in localization mode.'),
        DeclareLaunchArgument('rviz_cfg', default_value=config_rviz, description='Configuration path of rviz2.'),

        # 设置 use_sim_time 为 False
        SetParameter(name='use_sim_time', value=False),

        # 启动 RealSense D435 相机
        IncludeLaunchDescription(
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
                'rgb_camera.profile': '640x360x30',
                'depth_module.profile': '640x360x30',
                'enable_color': 'true',
                'enable_depth': 'true',
            }.items(),
        ),

        # 静态 TF 变换：LiDAR (base_link -> unilidar_lidar)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'unilidar_lidar']
        ),

        # 静态 TF 变换：相机 (base_link -> camera_link)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.1', '0', '0.3', '0', '0', '0', 'base_link', 'camera_link']
        ),

        # 节流点云：3Hz
        Node(
            package='topic_tools',
            executable='throttle',
            output='screen',
            arguments=['messages', '/unilidar/cloud', '3.0', '/unilidar/cloud_throttled']
        ),

        # ICP 里程计
        Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            output='screen',
            parameters=[parameters_odom],
            remappings=[('scan_cloud', '/unilidar/cloud_throttled')]
        ),

        # RGB-D 同步
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            output='screen',
            parameters=[parameters_slam, {
                'rgb_image_transport': 'raw',
                'depth_image_transport': 'raw',
                'approx_sync_max_interval': 0.5,
                'qos': 2,
                'sync_queue_size': 200
            }],
            remappings=remappings
        ),

        # SLAM 模式
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=[parameters_slam],
            remappings=remappings,
            arguments=['-d']
        ),

        # 定位模式
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=[parameters_slam, {
                'Mem/IncrementalMemory': 'False',
                'Mem/InitWMWithAllNodes': 'True'
            }],
            remappings=remappings
        ),

        # RTAB-Map 可视化
        Node(
            condition=IfCondition(rtabmap_viz),
            package='rtabmap_viz',
            executable='rtabmap_viz',
            output='screen',
            parameters=[parameters_slam],
            remappings=remappings
        ),

        # RViz
        Node(
            condition=IfCondition(rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_cfg')]
        ),
    ])