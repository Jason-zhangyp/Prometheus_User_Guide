memo:

本教程的目的：  

 - 让用户理解什么是建图？

 - RtabMap包的使用

 - 如何使用RtabMap算法进行GAZEBO仿真？

 - 如何进行真实的实验？

   Rtabmap_ros代码的总体说明（有什么用，使用了什么算法之类的介绍文字）

   详细的使用说明（包括launch文件参数的说明，选用不同输入来源时的用法）

   （原理+订阅的topic+发布的topic+参数）





### 相关launch文件介绍

手持建图（在gazebo中使用我们自己的worlds，手动飞行建图的流程，刚好把obstacle那个建一个完整的图）


 - 运行launch文件
   `roslaunch prometheus_gazebo sitl_mapping.launch`

 - 测试此功能时，无需自主飞行，此处推荐键盘控制或通过遥控器控制无人机 移动

 - 在rviz可观察建图情况

 - 更多详细信息：演示视频

   

   topic

   说明不需要slam

   选择：数据集or真机 选择

   rgbd数据集

   more 图

   参数+lanuch

   

﻿# 如何使用RTAB-Map进行建图

## 1. RTAB-Map介绍

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



## 2. RTAB-Map的使用

### 2.1 RTAB-Map安装

RTAB-Map可支持不同操作系统的安装，包括Ubuntu、Mac OS 和Windows，同时支持ROS，详见[安装](https://github.com/introlab/rtabmap/wiki/Installation)，本项目主要使用了RTAB-Map的ROS包，因此仅对ROS下的安装进行说明，

#### 2.1.1 二进制文件安装：

- Ubuntu 18.04版本

  ```
  $ sudo apt-get install ros-melodic-rtabmap-ros
  ```

- Ubuntu 16.04版本

  ```
  $ sudo apt-get install ros-kinetic-rtabmap-ros
  ```

#### 2.1.2 源码安装

对RTAB-Map进行源码编译，详见官方教程[编译](https://github.com/introlab/rtabmap_ros#rtabmap_ros-)

### 2.2 RTAB-Map的ROS节点

对于RTAB-MAP在ROS下的运行，详细可阅读[wiki](http://wiki.ros.org/rtabmap_ros)，本部分仅对主要的node的进行说明。

#### **2.2.1 rtabmap**：

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



#### 2.2.2 rtabmapviz

该节点启动RTAB-Map的界面，可以认为和rviz有类似的功能，该功能可以选择不开启

#### **2.2.3 rgbd_odometry**

rgbd的里程计，通过在终端运行如下命令来查看输入参数：

```
$ rosrun rtabmap_ros rgbd_odometry --params
```

#### **2.2.4 stereo_odometry**

双目的里程计，通过在终端运行如下命令来查看输入参数：

```
$ rosrun rtabmap_ros stereo_odometry --params
```

#### **2.2.5 icp_odometry**

激光的里程计，使用icp的方法进行定位，通过在终端运行如下命令来查看输入参数

```
$ rosrun rtabmap_ros icp_odometry --params
```



## 3 RTAB-Map测试

本部分介绍RTAM-Map的测试，分为三部分，分别是使用双目进行建图; 使用RGBD相机进行建图的测试以及在仿真环境下的建图，包含运行的方法，参数设置及结果演示。其中，双目的建图是在数据集下进行的；而RGBD的建图是使用realsense真机进行的测试，仿真环境下使用的相机模型是D435i。

### 3.1 双目建图

针对室外场景，RTAB-Map可以支持使用双目进行建图，下面介绍使用数据集的方式进行实现，参考官方教程[StereoMapping](http://wiki.ros.org/rtabmap_ros/Tutorials/StereoOutdoorMapping)。

#### 3.1.1 配置

[![GloYPx.md.jpg](https://s1.ax1x.com/2020/04/01/GloYPx.md.jpg)](https://imgchr.com/i/GloYPx)

#### 3.1.2 ROS bags下载

- [stereo_outdoorA.bag](https://docs.google.com/uc?export=download&confirm=ldd7&id=0B46akLGdg-uaOWtYT1ladldiS0U) 

- [stereo_outdoorB.bag](https://docs.google.com/uc?export=download&confirm=5Ptp&id=0B46akLGdg-uaVEs2SGFzT3ZVSU0)

#### 3.1.3 Launch

```
$ roslaunch rtabmap_ros demo_stereo_outdoor.launch
$ rosbag play --clock stereo_outdoorA.bag
[...]
$ rosbag play --clock stereo_outdoorB.bag
```

详细的[demo_stereo_outdoor.launch](https://github.com/introlab/rtabmap_ros/blob/master/launch/demo/demo_stereo_outdoor.launch) 

在双目的launch文件中，总共进行了三步，分别运行了stereo_image_proc对左右目图像作了去畸变处理:

```
   <group ns="/stereo_camera" >
      <node pkg="nodelet" type="nodelet" name="stereo_nodelet"  args="manager"/>
   
      <node pkg="stereo_image_proc" type="stereo_image_proc" name="stereo_image_proc">
         <remap from="left/image_raw"    to="left/image_raw_throttle_relay"/>
         <remap from="left/camera_info"  to="left/camera_info_throttle"/>
         <remap from="right/image_raw"   to="right/image_raw_throttle_relay"/>
         <remap from="right/camera_info" to="right/camera_info_throttle"/>
         <param name="disparity_range" value="128"/>
      </node>
   </group>
```

运行双目里程计:

```
   <!-- Stereo Odometry -->   
   <node pkg="rtabmap_ros" type="stereo_odometry" name="stereo_odometry" output="screen">
	...
   </node>
```

运行rtabmap建图：

```
   <node name="rtabmap" pkg="rtabmap_ros" type="rtabmap" output="screen" args="--delete_db_on_start">
    ...
   </node>
```

运行窗口：

```
   <node if="$(arg rtabmapviz)" pkg="rtabmap_ros" type="rtabmapviz" name="rtabmapviz" args="-d $(find rtabmap_ros)/launch/config/rgbd_gui.ini" output="screen">
   ...
   </node>
```

#### 3.1.4 运行结果

[![G1SSTf.md.png](https://s1.ax1x.com/2020/04/01/G1SSTf.md.png)](https://imgchr.com/i/G1SSTf)

（地图保存方式待补充）

### 3.2 RGBD建图

#### 3.2.1 真实环境下的建图

##### 相关相机介绍

RGBD真实环境下建图使用了英特尔的realsense的d435i和t265进行建图，其中d435i为建图提供深度图，而t265则提供实时的位姿给RTAB-Map，以这二者作为输入，RTAB-Map会生成相应的地图。

![GMhOD1.png](https://s1.ax1x.com/2020/03/31/GMhOD1.png)

两个相机的技术参数可查阅英特尔的官方文档，其中D435i的相关介绍在[D400-Series](https://www.intelrealsense.com/wp-content/uploads/2019/10/Intel-RealSense-D400-Series-Datasheet-Oct-2019.pdf?_ga=2.25926909.1365445393.1585639835-74784697.1581310827)以及[stereo depth product page](https://www.intelrealsense.com/stereo-depth/?_ga=2.190410731.1365445393.1585639835-74784697.1581310827);t265的相关介绍在[tracking camera datasheet](https://dev.intelrealsense.com/docs/tracking-camera-t265-datasheet)和[tracking product page](https://www.intelrealsense.com/tracking/) 。

##### T265和D435在空间上的对齐

为了将深度相机坐标系下的点云正确的转换到世界坐标系下，需要知道深度相机和机体坐标系（位姿坐标系）之间的转换关系，以及机体坐标系在世界坐标系下的位姿。两个相机之间的连接，采用了英特尔官方的demo实现，使用3D打印制作了两个相机的安装基座，并给定了二者之间的坐标转换关系，具体的教程详见[Tracking and Depth](https://dev.intelrealsense.com/docs/depth-and-tracking-cameras-alignment).  深度相机在机体位姿坐标系下的转换由H_pose_depth表示，而位恣坐标系在世界坐标系下的转换由H_world_pose表示，转换关系如下图所示：

![GQS59J.jpg](https://s1.ax1x.com/2020/03/31/GQS59J.jpg)

##### 硬件安装配置

D435i相机和T265相机的安装底座可以从[mount](https://github.com/IntelRealSense/librealsense/blob/development/examples/tracking-and-depth/bracket_t265nd435_external.stl)处下载，需要两个M3×10mm的螺栓安装D435i，还需要两个M3×18的螺栓安装T265。二者之间的转换关系配置文件详见[config](https://github.com/IntelRealSense/librealsense/blob/development/examples/tracking-and-depth/H_t265_d400.cfg)。

![GMzDL6.jpg](https://s1.ax1x.com/2020/03/31/GMzDL6.jpg)

##### 实际运行

首先按照官方教程对librealsense和realsense-ros进行安装：[Installation Instructions](https://github.com/IntelRealSense/realsense-ros). 安装完成后运行如下命令，分别启动D435i和T265

```
roslaunch realsense2_camera rs_d400_and_t265.launch
```

其中，launch中的tf节点：

```
<node pkg="tf" type="static_transform_publisher" name="t265_to_d400" args="-0.009375589 0.01590727 0.028273059 0 0 0 /$(arg tf_prefix_camera1)_link /$(arg tf_prefix_camera2)_link 100"/>
```

的作用就是发布D435i和T265相机坐标系之间的转换关系，其位置关系可以根据上面的config文件获得。

然后再运行rtab-map的节点，便可以建图了：

```
roslaunch realsense2_camera rs_rtabmap.launch
```

运行完效果如下（待补充）：

![GQuqbD.gif](https://s1.ax1x.com/2020/03/31/GQuqbD.gif)

### 3.3 仿真环境下的建图

#### 3.3.1 仿真环境及传感器配置



#### 3.3.2 RTAB-Map仿真配置





### 



