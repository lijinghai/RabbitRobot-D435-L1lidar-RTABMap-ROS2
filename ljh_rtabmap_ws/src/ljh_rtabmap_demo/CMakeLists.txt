cmake_minimum_required(VERSION 3.5)
project(ljh_rtabmap_demo)

# 使用 ament_cmake
find_package(ament_cmake REQUIRED)

# 依赖包（必须和 package.xml 保持一致）
find_package(rtabmap_ros REQUIRED)
find_package(realsense2_camera REQUIRED)
find_package(imu_filter_madgwick REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 安装 launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
