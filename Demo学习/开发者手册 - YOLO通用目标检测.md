# YOLO通用目标识别
  
本教程的目的：
 - 掌握YOLO通用目标检测程序，并能够进行二次开发
 - 学会训练自己的数据集
 - 学会使用YOLO程序的接口，完成相关的飞行任务


## 准备工作

#### opencv 3.3.1 安装

> 下载 opencv 3.3.1 源码，下载地址：http://121.36.68.10//upload/opencv-3.3.1.zip

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

![fig9](https://spire.imdo.co/images/2004/spire-tools-6.png)

在 label 中填写待标注目标名称，然后将对话框拖到一边
在主窗口中开始标注，鼠标滚轮放大缩小图像，按住左键移动可视图像区域不断点击左键将目标框包围，
使用 Yolo 训练时，点击 4 个点即可，不需要详细分割

![fig10](https://spire.imdo.co/images/2004/spire-tools-7.png)

![fig11](https://spire.imdo.co/images/2004/spire-tools-10.jpg)

![fig12](https://spire.imdo.co/images/2004/spire-tools-11.jpg)

标注时，如果点错，可以按 Ctrl+Z 撤销
标注完成后，如果不满意，可以点击绿色边框(边框会变红，如下图所示)，按Delete 删除

![fig13](https://spire.imdo.co/images/2004/spire-tools-12.jpg)

#### 将标注输出为 Yolo 格式，准备训练

在标注完成之后，按下 Ctrl+O

![fig14](https://spire.imdo.co/images/2004/spire-tools-13.jpg)

点击确定后

![fig15](https://spire.imdo.co/images/2004/spire-tools-14.jpg)

然后将下面 4 个文件取出用于 Yolo 训练

![fig16](https://spire.imdo.co/images/2004/spire-tools-15.jpg)


## 开始在 Yolo 上训练自己的数据


教程 darknet 路径为 /home/jario/darknet，本文以此为例。请根据自己的路径进行修改
在 /home/jario/darknet/cfg/ 文件夹下新建一个文件，名字叫 my_dataset.data 在里面写入：

```
classes = 1
train = /home/jario/darknet/data/coco/my_dataset_train.txt valid = /home/jario/darknet/data/coco/my_dataset_train.txt names = data/my_dataset.names
backup = backup eval=coco
```

> 注意：classes 为类别数量，对于单类检测问题，写 1

1. 在 /home/jario/darknet/data 中新建文件夹 coco
2. 将 Yolo_20180908_234114.txt 复制到 /home/jario/darknet/data/coco，并改名为 my_dataset_train.txt
3. 在 /home/jario/darknet/data/coco 中新建 2 个文件夹 images, labels
4. 将 scaled_images 文件夹 复制到 /home/jario/darknet/data/coco/images，并重命名为 train
5. 将 Yolo_labels 文件夹 复制到 /home/jario/darknet/data/coco/labels，并重命名为 train
6. 将 Yolo_categories.names 复制到 /home/jario/darknet/data，并重命名为my_dataset.names

将 /home/jario/darknet/cfg/yolov3-tiny.cfg 复制一份，重命名为 yolov3-tiny- train.cfg
打开刚刚重命名的文件/home/jario/darknet/cfg/yolov3-tiny-train.cfg 将前 7 行改为

```
[net]
# Testing # batch=1
# subdivisions=1 # Training batch=64 subdivisions=2
```

> 注意：对于显存比较小的用户，需要将 batch=64 改为 32 或 16

下载训练权重初值，https://pjreddie.com/media/files/darknet53.conv.74 ，并放到/home/jario/darknet 目录

准备已经完成，在终端下进入 /home/jario/darknet，目录用下面的命令开始训练

```
./darknet detector train cfg/my_dataset.data cfg/yolov3-tiny-train.cfg darknet53.conv.74
```

> 注意：如果出现如下错误

![fig17](https://spire.imdo.co/images/2004/spire-tools-20.jpg)

需要修改源码/home/jario/darknet/src/data.c 

将如下代码

```
list *get_paths(char *filename)
{
  char *path;
  FILE *file = fopen(filename, "r"); 
  if(!file) 
    file_error(filename); 
  list *lines = make_list(); 
  while((path=fgetl(file))) {
    list_insert(lines, path);
  }
  fclose(file); 
  return lines;
}
```

修改为

```
void ltrim(char *s)
{
char *p; p = s;
while (*p == ' ' || *p == '\t' || *p == '\r') { p++; } strcpy(s,p);
}


void rtrim(char *s)
{
int i;
i = strlen(s) - 1;
while ((s[i] == ' ' || s[i] == '\t' || s[i] == '\r') && i >= 0 ) { i--; } s[i+1] = '\0';
}

void _trim(char *s)
{
ltrim(s);
rtrim(s);
}

list *get_paths(char *filename)
{
char *path;
FILE *file = fopen(filename, "r"); if(!file) file_error(filename); list *lines = make_list(); while((path=fgetl(file))){
_trim(path); list_insert(lines, path);
}
fclose(file); return lines;
}
```


保存，make -j8 重新编译

等待训练完成，训练结果会保存在 /home/jario/darknet/backup 中

下面为训练时画面

![fig18](https://spire.imdo.co/images/2004/spire-tools-21.jpg)

## 如何进行真机实验？  

待补充  
  

