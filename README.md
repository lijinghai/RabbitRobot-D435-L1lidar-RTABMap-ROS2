<h1 align="center">
  🐇 RabbitRobot-D435-L1lidar-RTABMap-ROS2  
</h1>

<h3 align="center">
  RTAB-Map 融合 LiDAR 与双目相机的建图导航
</h3>

<p align="center">
  <img src="docs/demo.gif" width="600" alt="SLAM Demo"/>
</p>

> [!note]
>
> &emsp;&emsp;本项目基于 LiDAR、RGB-D 相机与 IMU 的多模态融合技术，为实现了移动小车的 三维建图与自主导航为目标。
系统采用 Unitree L1 激光雷达 提供高精度几何结构信息，结合 Intel RealSense D435 相机 提供彩色与深度数据，并融合 MPU6050 惯性测量单元 (IMU) 实现姿态与运动状态的补偿与优化。
通过 FAST-LIO2 与 RTAB-Map 的多源融合建图算法，系统能够在复杂环境下实现高精度的环境感知、鲁棒的位姿估计与实时自主导航功能，为 多传感器融合 SLAM 与智能移动机器人研究 提供了一个可靠的实验与开发平台。
>
> ​&emsp;&emsp;This project implements a **multi-modal fusion system** for 3D mapping and autonomous navigation using both **LiDAR and RGB-D camera** sensors. The system integrates the **Unitree L1 LiDAR** for high-precision geometric perception and the **Intel RealSense D435** for color and depth sensing. By combining **FAST-LIO2** and **RTAB-Map** frameworks, it achieves accurate environment reconstruction and robust navigation in real time, providing a solid foundation for research on **multi-sensor SLAM and autonomous mobile robots**.
