<h1 align="center">
  🐇 RabbitRobot-D435-L1lidar-RTABMap-ROS2  
</h1>

<h3 align="center">
  RTAB-Map 融合 LiDAR 与双目相机的建图导航
</h3>


> [!note]
>
> &emsp;&emsp;本项目基于 LiDAR、RGB-D 相机与 IMU 的多模态融合技术，为实现了移动小车的 三维建图与自主导航为目标。系统采用 Unitree L1 激光雷达 提供高精度几何结构信息，结合 Intel RealSense D435 相机 提供彩色与深度数据，并融合 MPU6050 惯性测量单元 (IMU) 实现姿态与运动状态的补偿与优化。
&emsp;&emsp;通过 FAST-LIO2 与 RTAB-Map 的多源融合建图算法，系统能够在复杂环境下实现高精度的环境感知、鲁棒的位姿估计与实时自主导航功能，为 多传感器融合 SLAM 与智能移动机器人研究 提供了一个可靠的实验与开发平台。
>
> ​&emsp;&emsp;This project integrates LiDAR (Unitree L1), RGB-D camera (Intel RealSense D435), and IMU (MPU6050) to achieve 3D mapping and autonomous navigation for mobile robots.
&emsp;&emsp;Using FAST-LIO2 and RTAB-Map, the system fuses geometric, visual, and inertial information to provide robust localization and environmental understanding even in complex scenarios, offering a reliable platform for multi-sensor SLAM research and intelligent robotics.
