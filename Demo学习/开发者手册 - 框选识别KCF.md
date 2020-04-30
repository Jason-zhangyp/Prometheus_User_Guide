# 框选识别KCF
  
本教程的目的：
 - 理解框选识别程序背后的原理
 - 掌握框选识别程序，并能够进行二次开发
 - 学会使用控制接口，完成与识别追踪相关的飞行任务


## 识别算法介绍

 - 订阅：

真实环境中订阅的相机话题为`/prometheus/camera/rgb/image_raw`

仿真环境中订阅的相机话题为`/P300_Monocular_front/Monocular/image_raw`
 
 - 发布：

检测结果话题`/prometheus/target` (话题格式请参考Prometheus/Modules/msgs/msg/DetectionInfo.msg)
 
 - 配置文件

真实环境中的配置文件为`Prometheus/Modules/object_detection/config/camera_param.yaml`

仿真环境中的配置文件为`Prometheus/Modules/object_detection/config/camera_param_gazebo_monocular.yaml`

注意修改kcf_tracker_h（跟踪框的实际高度）

- kcf算法介绍

KCF作为单目标跟踪的经典之作，在准确率和实时性上都有非常不错的表现，特别对算力要求不高。

算法亮点：

1. 通过循环矩阵生成正负样本来训练脊回归分类器；
2. 利用循环矩阵可DFT对角化的性质，将循环矩阵的求逆运算转化为向量的点乘；
3. 针对线性不可分的情况，引入核技巧映射到高维，线性可分

具体可以参考[博客](https://blog.csdn.net/shenxiaolu1984/article/details/50905283)

## 任务程序介绍
**object_tracking.cpp**

 - 订阅来自图像的识别信息，这里使用了自定义的消息`prometheus_msgs::MultiDetectionInfo`，具体消息格式可以在`msgs`文件夹中查看
 
 		`ros::Subscriber vision_sub = nh.subscribe<prometheus_msgs::MultiDetectionInfo>("/prometheus/target", 10, vision_cb);`
        
- 执行追踪算法及追踪策略（请阅读源码进行学习）
- 发布上层控制指令

		ros::Publisher command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

- 参数`Thres_vision`为视觉丢失阈值，`kpx_track/kpy_track/kpz_track`为控制参数，`start_point_x/start_point_y/start_point_z/start_yaw`为起始点位置，参数`tracking_delta_x/tracking_delta_y/tracking_delta_z`为追踪的前后间隔

## 仿真流程
- 运行启动脚本，注意修改参数配置文件`object_tracking.yaml`
    	roslaunch prometheus_gazebo sitl_kcf_detection.launch
- 启动后，输入1启动任务，飞机将起飞至起始点，等待目标信息
	[![GreO4H.md.png](https://s1.ax1x.com/2020/04/05/GreO4H.md.png)](https://imgchr.com/i/GreO4H)
- 在弹出来的交互框中使用鼠标左键框选追踪目标，使用鼠标右键取消框选
	[![GreKtH.md.png](https://s1.ax1x.com/2020/04/05/GreKtH.md.png)](https://imgchr.com/i/GreKtH)
- 飞机将飞向框选目标，完成任务


## 如何进行真机实验？  

待补充  
  

