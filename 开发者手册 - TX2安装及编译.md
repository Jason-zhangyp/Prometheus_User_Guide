# TX2安装及编译

## GPU加速版本的dataflow-orbslam

参考https://github.com/xaldyz/dataflow-orbslam， 论文

考虑到最新版JetPack已经使用OpenCv4，可选用该版本安装：https://github.com/jobesu14/dataflow-orbslam/tree/opencv4_support  需要于opencv-contrib modules一起编译，安装命令：https://github.com/AastaNV/JEP/blob/master/script/install_opencv4.1.1_Jetson.sh

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

### 编译过程中问题及解决

https://blog.csdn.net/AlexWang30/article/details/99612188