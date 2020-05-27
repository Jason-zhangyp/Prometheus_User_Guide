# VSLAM之maplab篇

## maplab介绍

[maplab](https://github.com/ethz-asl/maplab)是ETH的ASL团队在2018年开源的一个视觉+惯性导航的建图框架，可以实现多场景建图的创建、处理和操作。它包含了两部分：在线的前端ROVIOLI用于创建地图，可以实现建图和定位功能；离线的地图处理包maplab_console，可以为生成的地图进行二次处理（生成地图优化、组合和建立稠密地图等）。其工作流程如下图所示

![tAMUG4.png](https://s1.ax1x.com/2020/05/27/tAMUG4.png)

- (a)  在VIO模式下使用ROVIOLI进行建图。

- (b)  使用maplab console对地图进行优化，实现不同地图的融合。

- (c)  在定位模式下运行ROVIOLI，定位的功能提高了视觉+惯性导航的位姿估计精度。

## maplab的编译

按照[官方教程](https://github.com/ethz-asl/maplab/wiki/Installation-Ubuntu)进行编译，

- 首先安装依赖

需要注意，clang-format-3.8已经无法在18.04的ubuntu上使用了，需要安装3.9版本.

```
# Install ROS 
export UBUNTU_VERSION=bionic #(Ubuntu 16.04: xenial, Ubuntu 14.04: trusty, Ubuntu 18.04: bionic)
export ROS_VERSION=melodic #(Ubuntu 16.04: kinetic, Ubuntu 14.04: indigo, Ubuntu 18.04: melodic)

# NOTE: Follow the official ROS installation instructions for melodic.
sudo apt install software-properties-common
sudo add-apt-repository "deb http://packages.ros.org/ros/ubuntu $UBUNTU_VERSION main"
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
sudo apt update
sudo apt install ros-$ROS_VERSION-desktop-full "ros-$ROS_VERSION-tf2-*" "ros-$ROS_VERSION-camera-info-manager*" --yes


# Install framework dependencies.
# NOTE: clang-format-3.8 is not available anymore on bionic, install a newer version.
sudo apt install autotools-dev ccache doxygen dh-autoreconf git liblapack-dev libblas-dev libgtest-dev libreadline-dev libssh2-1-dev pylint clang-format-3.9 python-autopep8 python-catkin-tools python-pip python-git python-setuptools python-termcolor python-wstool libatlas3-base --yes

sudo pip install requests
```

- 然后更新ROS环境

```
sudo rosdep init
rosdep update
echo ". /opt/ros/$ROS_VERSION/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

- 创建catkin工作空间

```
export ROS_VERSION=kinetic #(Ubuntu 16.04: kinetic, Ubuntu 14.04: indigo)
export CATKIN_WS=~/maplab_ws
mkdir -p $CATKIN_WS/src
cd $CATKIN_WS
catkin init
catkin config --merge-devel # Necessary for catkin_tools >= 0.4.
catkin config --extend /opt/ros/$ROS_VERSION
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
cd src
```

- 下载

```
git clone https://github.com/ethz-asl/maplab.git --recursive
git clone https://github.com/ethz-asl/maplab_dependencies --recursive
```

- 编译：

```
cd $CATKIN_WS
catkin build maplab
```

- 编译maplab遇到问题及相应解决方法：

1. 下载依赖遇到问题：

   当编译依赖时，会首先下载源码，有时网络不好的话会导致下载失败，进而导致编译失败，解决方法可以参照官方[FAQ](https://github.com/ethz-asl/maplab/wiki/FAQ#q-why-do-i-get-missing-dependencies-when-building-the-maplab-workspace)，首先离线下载源码，然后copy到相应位置，比如opencv_catkin，可以从

   ```
   https://github.com/Itseez/opencv/archive/3.2.0.zip
   ```

   中下载，然后将源码解压至：

   ```
   ~/maplab_ws/build/opencv3_catkin/opencv3_src
   ```

   

2. 编译opencv3_catkin遇到问题~/maplab_ws/build/opencv3_catkin/opencv3_src/cmake/OpenCVCompilerOptions.cmake报以下错误：

   ```
   A duplicate ELSE command was found inside an IF block.
   ```

   在OpenCVCompilerOptions.cmake文件中注释掉报错的对应行即可。

3. 遇到brisk-detector源码问题：解决方法参照https://github.com/ethz-asl/ethzasl_brisk/pull/109，在brisk-feature-detector.cc中加入包含#include <functional>