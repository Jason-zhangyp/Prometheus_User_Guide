# YOLO通用目标识别
  
本教程的目的：
 - 掌握YOLO通用目标检测程序，并能够进行二次开发
 - 学会训练自己的数据集
 - 学会使用YOLO程序的接口，完成相关的飞行任务


## 准备工作

#### opencv 3.3.1 安装

> 下载 opencv 3.3.1 源码，下载地址：http://192.168.1.212/upload/opencv-3.3.1.zip

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

#### 安装NVIDIA显卡驱动，CUDA与CUDNN

1. 官方教程 [https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#ubuntu-installation](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#ubuntu-installation)

2. 在 [http://developer.nvidia.com/cuda-downloads](http://developer.nvidia.com/cuda-downloads) 上下载安装包

![fig1](https://spire.imdo.co/images/2004/1656228-20190709172033436-1570003711.png)

3. 到安装文件目录下运行.run文件，输入accept

```
sudo sh cuda_10.1.168_418.67_linux.run
```

4. 根据自身选择要不要安装Nvidia显卡驱动

![fig2](https://spire.imdo.co/images/2004/1656228-20190709172606285-197126065.png)

因为我们只是调用CUDA，不去写CUDA程序，所以Samples也可以不安装

![fig3](https://spire.imdo.co/images/2004/1656228-20190709172730248-1234756893.png)


5. 添加环境变量 

```
vi ~/.bashrc
# 在文件末尾添加
export PATH="/usr/local/cuda-10.1/bin:$PATH"
export LD_LIBRARY_PATH="/usr/lcoal/cuda-10.1/lib64:$LD_LIBRARY_PATH"
# 最后使其生效
source ~/.bashrc
```


## 数据集标注

下载数据集标注工具，下载地址：[**Baidu Pan**](https://pan.baidu.com/s/1H7VmTgcVpyIv03H6AtjtoQ) (password: 30dl) 或者 [**Spire Web**](http://121.36.68.10/tools/ImageLabelTools-4.1.6.zip).

数据集管理软件github地址：https://github.com/jario-jin/spire-image-manager

#### 打开标注软件 SpireImageTools_x.x.x.exe

首先点击Tools->Setting...，填写一个 save path (所有的标注文件都会存储在这个文件夹中)

![fig4](https://spire.imdo.co/images/2004/spire-tools-1.png)

如果采集的数据集是视频 (**如果采集的是图像，则调过这一步骤**)，点击 Input->Video， 选择要标注的视频。

![fig5](https://spire.imdo.co/images/2004/spire-tools-2.png)

然后，点击Tools->Video to Image

![fig6](https://spire.imdo.co/images/2004/spire-tools-3.png)

点击OK 后，等待完成，结果会存储在

![fig7](https://spire.imdo.co/images/2004/spire-tools-4.png)

#### 打开需要标注的图像

Input->Image Dir， 找到需要标注的图像所在文件夹 Ctrl+A，全选，打开

![fig8](https://spire.imdo.co/images/2004/spire-tools-5.png)



点击，Tools->Annotate Image->Instance Label，开始标注图像




## 如何进行真机实验？  

待补充  
  

