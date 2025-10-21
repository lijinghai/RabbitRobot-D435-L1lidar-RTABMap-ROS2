import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    parameters = [{
        'frame_id': 'camera_link',
        'subscribe_depth': True,
        'approx_sync': False,
    }]

    remappings = [
        ('rgb/image', '/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
        ('depth/image', '/camera/aligned_depth_to_color/image_raw')
    ]

    return LaunchDescription([
        # 启动参数
        DeclareLaunchArgument(
            'args', default_value='',
            description='Extra arguments set to rtabmap and odometry nodes.'
        ),

        DeclareLaunchArgument(
            'odom_args', default_value='',
            description='Extra arguments just for odometry node.'
        ),

        # 启动 RealSense 相机驱动
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
                'enable_depth': 'true'
            }.items(),
        ),

        # 启动 RTAB-Map 里程计
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            output='screen',
            parameters=parameters,
            arguments=[LaunchConfiguration("args"), LaunchConfiguration("odom_args")],
            remappings=remappings
        ),

        # 启动 RTAB-Map SLAM 主节点
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d', LaunchConfiguration("args")]
        ),

        # 启动可视化界面
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            output='screen',
            parameters=parameters,
            remappings=remappings
        ),
    ])
