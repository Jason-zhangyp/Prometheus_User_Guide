## 

﻿# 如何使用RTAB-Map进行建图

## ROS 中激光及视觉建图的总结

### 激光SLAM框架

### 视觉SLAM框架

本教程的目的：  

 - 让用户理解什么是建图？
 - RtabMap包的使用
 - 如何使用RtabMap算法进行GAZEBO仿真？
 - 如何进行真实的实验？

## RTAB-Map的使用

### RTAB-Map介绍

[RTAB-Map](http://introlab.github.io/rtabmap/) (Real-Time Appearance-Based Mapping)是一种基于增量式外观特征进行回环检测的SLAM框架，可以支持包括RGB-D，双目和激光雷达在内的多种传感器进行定位和建图。RTAB-Map使用词袋的方式判断新接收到的图像有多大可能是来自一个新位置或是曾经到过的位置，以此来完成回环检测，当检测到回环时，新的约束会被加到地图中，然后通过图优化的方式来最小化误差。RTAB-Map回使用内存管理的方法，限制用于回环检测和图优化的位置数量以保证在大规模环境中的实时性。RTAB-Map可以使用手持的RGB-D相机、双目相机或三维激光雷达进行六自由度建图。

自2013年开源以来，RTAB-Map已经发展成为了一个较为全面的SLAM框架，可以在不同类型的传感器下使用，并且已经发展成了一个独立的跨平台的C++库和ROS的包，可以满足实时性、鲁棒的里程计和定位功能，不同用途的建图及重定位功能等需求。以下图片展示了rtabmap的ROS节点的框图。需要的输入有：

- **TF：**传感器位置和机器人本体之间的位置关系

- **Odometery：**3DoF或者6DoF的里程计，可以使任意来源

- **相机输入：**可以是双目或者是RGB-D，带有相应的标定信息

产生的输出有：

- **TF：**里程及纠正信息
- **OctoMap： **三维占用地图（可选）
- **Point Cloud：**三维稠密点云
- **Ocuupancy Grid：**二维占用地图
- **Map Data：**包含传感器信息的地图数据
- **Map Graph：**不包含数据，只包含图

![GizrVK.png](https://s1.ax1x.com/2020/03/27/GizrVK.png)

##### 

### RTAB-Map安装

RTAB-Map可支持不同操作系统的安装，包括Ubuntu、Mac OS 和Windows，同时支持ROS，详见[安装](https://github.com/introlab/rtabmap/wiki/Installation)，本项目主要使用了RTAB-Map的ROS包，因此仅对ROS下的安装进行说明，

##### 二进制文件安装：

- Ubuntu 18.04版本

  ```
  $ sudo apt-get install ros-melodic-rtabmap-ros
  ```

- Ubuntu 16.04版本

  ```
  $ sudo apt-get install ros-kinetic-rtabmap-ros
  ```

##### 源码安装

#### 同时也可以对RTAB-Map进行源码编译，详见[编译](https://github.com/introlab/rtabmap_ros#rtabmap_ros-)

### RTAB-Map的ROS节点

对于RTAB-MAP在ROS下的运行，详细可阅读[wiki](http://wiki.ros.org/rtabmap_ros)，本部分仅对主要的node的进行说明。

#### **rtabmap**：

这是RTAM-Map的ROS包的核心节点，包含了RTAB-Map的核心库，地图的增量式构建、地图优化和回环检测就是在这个节点内进行的。通过订阅cloud_map，grid_map（proj_map）的话题，可以获得三维点云和二维占据地图。另外，RTAB-Map的数据库存储在路径"~/.ros/rtabmap.db"下，可以通过指令"--delete_db_on_start"在每次启动时进行删除，否则将会加载前一次的数据库。

rtabmap这个节点的所有参数可以通过参数头文件[Parameters.h](https://github.com/introlab/rtabmap/blob/master/corelib/include/rtabmap/core/Parameters.h#L161)查看，或者通过运行以下命令在terminal查看：

```
$ rosrun rtabmap_ros rtabmap --params
```

##### 注意：

默认状态下，rtabmap是在建图模式下运行，如果要在之前创建的地图下运行定位模式，则需要将rtabmap设置为非增量，同时也要保证参数“--delete_db_on_start”不包含在指令中：

```
<launch>
<node name="rtabmap" pkg="rtabmap_ros" type="rtabmap" args="">
   <!-- LOCALIZATION MODE -->
   <param name="Mem/IncrementalMemory" type="string" value="false"/>
</node>
</launch>
```

##### 订阅话题

- 里程计：如果参数subscribe_depth或者参数subscribe_stereo是true的话，或者里程计坐标系odom_frame_id没有被设置的话，则需要订阅此里程计话题-odom([nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)) 

- 图像：

  单目RGB图像：rgb/image ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)) 

  单目RGB相机参数：rgb/camera_info ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html));

  深度图像：depth/image([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)) 

  RGBD图像：rgbd_image([rtabmap_ros/RGBDImage](http://docs.ros.org/api/rtabmap_ros/html/msg/RGBDImage.html)) 

- 激光：

  激光扫描：scan ([sensor_msgs/LaserScan](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html)) 

  激光点云：scan_cloud ([sensor_msgs/PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html)) 

- 双目：

  左目矫正图像：left/image_rect ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)) 

  左目相机信息：left/camera_info ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)) 

  右目矫正图像：right/image_rect ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)) 

  右目相机信息：right/camera_info ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)) 

- 目标点信息：goal ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html)) 用于规划全局路径

##### 发布话题

- 地图信息：

  RTAM-Map系统信息：info([rtabmap_ros/Info](http://docs.ros.org/api/rtabmap_ros/html/msg/Info.html)) 	

  地图数据：mapData ([rtabmap_ros/MapData](http://docs.ros.org/api/rtabmap_ros/html/msg/MapData.html)) 

  地图的图：mapGraph ([rtabmap_ros/MapGraph](http://docs.ros.org/api/rtabmap_ros/html/msg/MapGraph.html))

- 地图：

  激光扫描建立的占据图：grid_map ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html)) 

  三维点云投影到地面上的占据图：proj_map ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html)) 

  三维点云图：cloud_map ([sensor_msgs/PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html)) 

  八叉树地图：octomap_full ([octomap_msgs/Octomap](http://docs.ros.org/api/octomap_msgs/html/msg/Octomap.html)) 

  八叉树地图：octomap_binary ([octomap_msgs/Octomap](http://docs.ros.org/api/octomap_msgs/html/msg/Octomap.html)) 

  八叉树中占据空间的点云图：octomap_occupied_space ([sensor_msgs/PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html)) 

  八叉树中障碍物的点云图octomap_obstacles ([sensor_msgs/PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html)) 

  八叉树中地面的点云图：octomap_ground ([sensor_msgs/PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html)) 

  八叉树中空白空间的点云图octomap_empty_space ([sensor_msgs/PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html)) 

  将八叉树投影到二维平面的占据图：octomap_grid ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html)) 

- 规划

  RTAM-Map中有基于图的路径规划，但是不在本部分考虑范围内，就不详细介绍了

##### 参数



#### rtabmapviz

该节点启动RTAB-Map的界面，可以认为和rviz有类似的功能，该功能可以选择不开启

#### **rgbd_odometry**

#### **stereo_odometry**

#### **icp_odometry**

**其他**

#### Launch



Rtabmap_ros代码的总体说明（有什么用，使用了什么算法之类的介绍文字）

详细的使用说明（包括launch文件参数的说明，选用不同输入来源时的用法）

（原理+订阅的topic+发布的topic+参数）



## RTAB-Map测试

### 数据集测试

数据集建图demo（使用官网的提供的数据集（备注下载链接，如何跑数据集跑出来建图的效果）

### 真机测试

本教程会对使用RGB-D相机和双目相机进行建图的例子进行详细说明。



## Gazebo仿真环境运行

### RTAB-Map仿真配置



### 相关launch文件介绍

手持建图（在gazebo中使用我们自己的worlds，手动飞行建图的流程，刚好把obstacle那个建一个完整的图）


 - 运行launch文件
	 `roslaunch prometheus_gazebo sitl_mapping.launch`
 - 测试此功能时，无需自主飞行，此处推荐键盘控制或通过遥控器控制无人机 移动
 - 在rviz可观察建图情况
 - 更多详细信息：演示视频

### 运行截图



