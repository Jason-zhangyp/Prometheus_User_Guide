# 使用FastPlanner进行轨迹优化

本教程的目的：  

 - 让用户理解什么是轨迹优化？什么是FastPlanner算法？
 - FastPlanner算法的原理解释，相关参数解释
 - 如何使用FastPlanner算法进行GAZEBO仿真？（三种情况：激光，RGBD，离线地图）
 - 如何进行真实的实验？
  
## 规划框架
FastPlanner 由三部分组成，分别包括混合A*算法、Bspline轨迹生成和自动时间分配调整。
混合A*算法用于
Bspline轨迹生成用于
自动时间分配用于
因此整个算法可以实现

## FastPlanner算法介绍 （原理+订阅的topic+发布的topic+参数）

通过什么代码，用了什么包，原始输入是什么，输出是什么。。  
发布了什么话题，这个话题的意义等等  


## Gazebo仿真环境运行  
  
  分三种情况来写，对应不同的指令及rviz设置

 - 运行launch文件  
  `roslaunch prometheus_gazebo sitl_fast_planning_rgbd.launch`  
 - 在rviz中指定目标点  
 - 或通过终端输入目标点  
 - 更多详细信息：演示视频  
  

### 运行截图  

## 真实环境中运行  
  

