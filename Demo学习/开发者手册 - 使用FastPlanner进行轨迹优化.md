# 使用FastPlanner进行轨迹优化
  
## 轨迹优化介绍


## FastPlanner算法介绍 （原理+订阅的topic+发布的topic+参数）

通过什么代码，用了什么包，原始输入是什么，输出是什么。。  
发布了什么话题，这个话题的意义等等  


## 规划算法任务程序
**planning_mission.cpp**

请查看局部规划算法文档中的介绍


## Gazebo仿真环境运行  
  
  使用激光雷达作为传感器
 - 运行launch文件（请查看launch文件参数说明，并进行调整）
  		roslaunch prometheus_gazebo sitl_fast_planning_3dlidar.launch 
 - 在打开的rviz窗口中勾选Fast_Planner、Octomap_Mapping及Ground_Truth显示
 - 输入3选择Fast Planner算法，无人机将自动起飞
 - 在rviz中通过3D Nav Goal按钮指定目标点，点选该按钮后，同时按住鼠标左右键在rviz窗口中选择一点向上拉  
    [![G66Zi6.png](https://s1.ax1x.com/2020/04/07/G66Zi6.png)](https://imgchr.com/i/G66Zi6)
 - 也可以通过终端发布目标点  
 		rostopic pub /prometheus/planning/goal ...
 - 通过终端查看算法相关信息
   [![G6cw9K.md.png](https://s1.ax1x.com/2020/04/07/G6cw9K.md.png)](https://imgchr.com/i/G6cw9K)
  
  使用RGBD相机作为传感器
 - 运行launch文件（请查看launch文件参数说明，并进行调整）
  		roslaunch prometheus_gazebo sitl_fast_planning_rgbd.launch 
 - 在打开的rviz窗口中勾选Fast_Planner及Ground_Truth显示
 - 其他同上

## 真实环境中运行  
  

