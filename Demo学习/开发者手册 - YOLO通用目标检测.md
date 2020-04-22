# YOLO通用目标识别
  
本教程的目的：
 - 掌握YOLO通用目标检测程序，并能够进行二次开发
 - 学会训练自己的数据集
 - 学会使用YOLO程序的接口，完成相关的飞行任务


## 准备工作

#### opencv 3.3.1 安装

- 下载 opencv 3.3.1 源码

> 下载地址：http://192.168.1.212/upload/opencv-3.3.1.zip

解压 opencv 3.3.1
```
cd opencv-3.3.1 
mkdir build
sudo apt-get install cmake 
cd build
cmake ..
```

> 注意 IPPICV: Download: ippicv_2017u3_lnx_intel64_general_20170822.tgz 
> 如果遇到下载问题，请多次重试

```
make -j8
sudo make install
```

- 安装NVIDIA显卡驱动，CUDA与CUDNN

1. 官方教程https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#ubuntu-installation

2. 在http://developer.nvidia.com/cuda-downloads上下载安装包

![nvidia cuda](https://spire.imdo.co/images/2004/1656228-20190709172033436-1570003711.png)

3. 到安装文件目录下运行.run文件，输入accept



## 识别算法介绍

**pytorch_mnist_camera.py及pytorch_mnist_gazebo.py** 
 - 订阅的相机话题为`/prometheus/camera/rgb/image_raw`
 - 订阅的相机话题为`/P300_Monocular_front/Monocular/image_raw`

 - 方向定义： 目标位置 [相机系下：右方x为正，下方y为正，前方z为正]
 - 默认发布话题：  /prometheus/target (话题格式请参考Prometheus/Modules/msgs/msg/DetectionInfo.msg)
 
**识别算法工作流程**

![G07ccT.png](https://s1.ax1x.com/2020/04/04/G07ccT.png)

- 数字识别网络——LeNet-5，结构如下图所示

[![GwfGjA.png](https://s1.ax1x.com/2020/04/04/GwfGjA.png)](https://imgchr.com/i/GwfGjA)

- 透视n点算法估计相对位姿

[![Gw4wlQ.png](https://s1.ax1x.com/2020/04/04/Gw4wlQ.png)](https://imgchr.com/i/Gw4wlQ)

**识别算法参数修改**

 - 首先启动相机节点，如需修改相机ID，请参考代码（使用gazebo仿真请忽略这一部分）

```
rosrun prometheus_detection web_cam
rosrun camera_calibration cameracalibrator.py --size 8x6 --sqre 0.0245 image:=/prometheus/camera/rgb/image_raw
# size为标点板尺寸，square为每个方格宽度(m)，image:=相机话题
```

 - 将得到的参数写入如下文件(有关数字板尺寸的预定义也在这个文件中：digitnum_det_len，单位m，为数字板的白色边长，检测时存在误差)

```
# 真实相机参数文件
Prometheus/Modules/object_detection/config/camera_param.yaml
# gazebo仿真环境中的相机参数问题，默认情况下不需要标定仿真环境中的相机
Prometheus/Modules/object_detection/config/camera_param_gazebo_monocular.yaml
```

- 数字板示例

![G0qmcV.png](https://s1.ax1x.com/2020/04/04/G0qmcV.png)

[**下载地址**](https://spire.imdo.co/upload/0-9nums.zip)

数字可以兼容大部分手写字体，但是一定要有黑框与白底的正方形区域包围

## 任务程序介绍
**number_detection.cpp**

 - 订阅来自图像的识别信息，这里使用了自定义的消息`prometheus_msgs::MultiDetectionInfo`，具体消息格式可以在`msgs`文件夹中查看
 
 		`ros::Subscriber vision_sub = nh.subscribe<prometheus_msgs::MultiDetectionInfo>("/prometheus/target", 10, vision_cb);`
        
 - 控制飞机自主起飞并悬停在数字墙前方，并打印显示识别结果。（注意：此处显示的结果为相机坐标系数值）
[![GwZKit.md.png](https://s1.ax1x.com/2020/04/04/GwZKit.md.png)](https://imgchr.com/i/GwZKit)

## 数字识别Gazebo仿真
**仿真world介绍**

  具体world配置内容，请查看`wall_num.world`及`wall_with_num.sdf`
  
  当飞机悬停于[0,0,1.5]位置，并正对数字墙时，每一个数字ID相对于飞机的距离为
   - 数字0：前方5米，左边2米，上方1米
   - 数字1：前方5米，左边1米，上方1米
   - 数字2：前方5米，左边0米，上方1米
   - 数字3：前方5米，右边1米，上方1米
   - 数字4：前方5米，右边2米，上方1米
   - 数字5：前方5米，左边2米，下方1米
   - 数字6：前方5米，左边1米，下方1米
   - 数字7：前方5米，左边0米，下方1米
   - 数字8：前方5米，右边1米，下方1米
   - 数字9：前方5米，右边2米，下方1米

注意：由于相机坐标系与机体坐标系不重合，以及数字ID和墙体厚度的原因，解算出来的相对位置并不精准。

  [![GwVEEn.md.png](https://s1.ax1x.com/2020/04/04/GwVEEn.md.png)](https://imgchr.com/i/GwVEEn)
  
**仿真流程**
 - 运行launch文件
	 `roslaunch prometheus_gazebo sitl_number_detection.launch`
 - 在`number_detection`终端中输入1开始任务，飞机将自动解锁，切换至OFFBOARD模式，并起飞至悬停点[0,0,1.5]
[![GwVFBj.md.png](https://s1.ax1x.com/2020/04/04/GwVFBj.md.png)](https://imgchr.com/i/GwVFBj)
 - 可在窗口中观察到数字的识别情况，也可以在终端中查看识别情况及相对距离。
[![GwVkHs.md.png](https://s1.ax1x.com/2020/04/04/GwVkHs.md.png)](https://imgchr.com/i/GwVkHs)

运行截图:
![GOkVbt.gif](https://s1.ax1x.com/2020/04/12/GOkVbt.gif)
## 如何进行真机实验？  

待补充  
  

