# TX2开发环境部署
虽然最新版JetPack已经使用OpenCV4，但是因为该版本的orbslam以及相机的sdk依然依赖OpenCV3，所以首先安装低版本的OpenCV，此处安装编译的版本是3.4.3版本，可以使用install_opencv3.4.3_Jetson.sh进行安装
## 编译过程中问题及解决

编译过程出现缺少boostdesc_bgm.i的解决方法：

https://blog.csdn.net/AlexWang30/article/details/99612188

编译过程出现无法打开包括文件: “opencv2/xfeatures2d/cuda.hpp”一类问题的的解决方法‘

将无法找到的头文件相对路径改为该文件的绝对路径，编译通过。

## GPU加速版本的dataflow-orbslam

参考https://github.com/xaldyz/dataflow-orbslam， 论文


### 设置JetPack

### 安装依赖

Opencv

#### Pangolin

用于用户界面和视觉化结果，下载和安装教程参照https://github.com/stevenlovegrove/Pangolin.

##### 依赖：

```
sudo apt install libgl1-mesa-dev
sudo apt install libglew-dev
sudo apt install cmake
```

##### 编译

```
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build
cd build
cmake ..
cmake --build .
```

#### Eigen

#### DBoW2 and g2o

### 源码编译

```
sudo apt-get update
sudo apt-get install git cmake
git clone https://github.com/xaldyz/dataflow-orbslam.git
cd ~/dataflow-orbslam/Thirdparty/DBoW2   # Build DBoW
mkdir build && cd build
cmake ..
make
cd ~/dataflow-orbslam/Thirdparty/g2o      # Build g2o
mkdir build && cd build
cmake ..
make
```

编译

```
cd ~/dataflow-orbslam
mkdir build && cd build
cmake ..
make
```



