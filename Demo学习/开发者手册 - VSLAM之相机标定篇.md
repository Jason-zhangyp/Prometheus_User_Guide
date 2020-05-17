# VSLAM相机配置篇

本篇主要介绍相机的类型，标定过程及标定结果。相机类型包含双目和RGBD相机，分别选用了MYNTEYE    

S1030-ir和realsense D435i两款

## 双目相机启动：Mynteye-S1030-ir

https://slightech.github.io/MYNT-EYE-S-SDK-Guide/src/slam/how_to_use_kalibr.html

双目选用的款式为小觅的标准款，从官方github下载[SDK](https://github.com/slightech/MYNT-EYE-S-SDK)，并进行编译：

```
make init
make all
```

编译完成后便可运行相应的测试例程

```
./samples/_output/bin/camera_with_junior_device_api`
```

或者运行ROS：

```
roscore
```

启动相机节点：

```
source ./wrappers/ros/devel/setup.bash
roslaunch mynteye_wrapper_d mynteye.launch
```

需要注意的是，如果选用的是小觅相机带红外的的标准款，则在image_raw图像中会出现一些条纹，可能会影响标定效果，需要设置参数进行修改，标准版的参数保存在路径

```
MYNT-EYE-S-SDK/wrappers/ros/src/mynt_eye_ros_wrapper/config/device/standard.yaml
```

文件中，可以对其中参数进行修改。将ir_control设置为0便可对其关闭，同时对于标定，可以将相机图像帧率进行降低，此处降至最低10hz

![YRrLC9.png](https://s1.ax1x.com/2020/05/17/YRrLC9.png)

## Kalibr的安装

按照官方[安装教程](https://github.com/ethz-asl/kalibr/wiki/installation)对kalibr进行源码编译

1. 安装ROS，在此不再赘述
2. 安装和编译依赖

```
sudo apt-get install python-setuptools python-rosinstall ipython  libeigen3-dev libboost-all-dev doxygen libopencv-dev  ros-melodic-vision-opencv ros-melodic-image-transport-plugins  ros-melodic-cmake-modules python-software-properties  software-properties-common libpoco-dev python-matplotlib python-scipy  python-git python-pip ipython libtbb-dev libblas-dev liblapack-dev  python-catkin-tools libv4l-dev 

sudo pip install python-igraph --upgrade
```

需要注意的是，官方安装教程较老，使用的ROS版本还是Indigo，所以需要将其替换为melodic，同时删除python-software-properties，因为新版系统也没有该软件可供安装。

3. 创造工作空间

```
mkdir -p ~/kalibr_workspace/src 
 cd ~/kalibr_workspace 
 source /opt/ros/indigo/setup.bash 
 catkin init 
 catkin config --extend /opt/ros/indigo 
 catkin config --merge-devel # Necessary for catkin_tools >= 0.4. catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
```

4. 编译

```
cd ~/kalibr_workspace/src 
git clone https://github.com/ethz-asl/Kalibr.git
cd ~/kalibr_workspace 
catkin build -DCMAKE_BUILD_TYPE=Release -j4
```

5. 刷新工作空间

```
source ~/kalibr_workspace/devel/setup.bash
```

## Kalibr使用

### 多相机标定

参照多相机标定的[官方例程](https://github.com/ethz-asl/kalibr/wiki/multiple-camera-calibration)对双目相机的内参和外参进行标定，Kalibr是使用ROSBAG进行标定的，所以首先采集图像。

1. 图像采集

将相机固定，移动标定板以获得标定图像，官方建议降低图像发布频率到4hz，使用ros throttle的工具可以实现对两个相机分别降频，但问题在于经过实测，这样降低频率会造成双目图像不同布，如下图所是：

[![YR25uR.md.png](https://s1.ax1x.com/2020/05/17/YR25uR.md.png)](https://imgchr.com/i/YR25uR)

所以在此不建议采用此方法，本次标定使用的方法是修改上述配置参数的方式，虽然只能降低至最低10hz，经测试并不影响效果，但实现了毫米级同步，对于最终的标定效果有力：

[![YRRSbt.md.png](https://s1.ax1x.com/2020/05/17/YRRSbt.md.png)](https://imgchr.com/i/YRRSbt)

```
rosbag record /mynteye/left/image_w /mynteye/right/image_raw -O stereo_calibration.bag
```

其中输入参数为左右目相机topic和保存的rosbag文件名

2. 进行标定

标定需要的文件：

- 包含数据的rosbag

- 所有相机的topic，要与--models中的顺序对应

- 相机的畸变模型，kalibr所支持的[畸变模型](https://github.com/ethz-asl/kalibr/wiki/supported-models)

- 标定版参数。此处使用的方格(checkboard)标定版，需要设置角点的xy方向的数量及方格尺寸

```
cd ~/kalibr_workspace
source devel/setup.bash
kalibr_calibrate_cameras --bag /home/colin/datasets/stereo_calibration.bag --topics /mynteye/left/image_raw /mynteye/right/image_raw --models pinhole-radtan pinhole-radtan --target /home/colin/datasets/checkerboard.yaml 
```

3. 输出文件

- **report-cam-%BAGNAME%.pd**：标定完成后数据会生成PDF文件，从重投影误差的效果来看标定的比较好。

[![YR4ot0.md.png](https://s1.ax1x.com/2020/05/17/YR4ot0.md.png)](https://imgchr.com/i/YR4ot0)

- **results-cam-%BAGNAME%.txt**：结果总结
- **camchain-%BAGNAME%.yaml**：此文件用于下一步camera-imu的标定

### IMU参数

此处需要使用IMU的datasheet进行设定，因暂时未查到，使用kalibr默认[配置](https://github.com/ethz-asl/kalibr/wiki/yaml-formats)

### Camera-IMU标定



## RGBD相机

