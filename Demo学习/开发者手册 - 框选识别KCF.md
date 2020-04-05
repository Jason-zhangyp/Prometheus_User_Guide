# 数字识别
  
本教程的目的：
 - 理解框选识别程序背后的原理
 - 掌握框选识别程序，并能够进行二次开发
 - 学会使用控制接口，完成与识别追踪相关的飞行任务


## 识别算法介绍

kcf介绍

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
  

