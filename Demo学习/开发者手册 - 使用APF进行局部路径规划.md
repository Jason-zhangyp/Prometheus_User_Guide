# 使用APF进行局部路径规划
  

本教程的目的：  

 - 让用户理解什么是局部路径规划？什么是APF算法？
 - APF算法的原理解释，相关参数解释
 - 如何使用APF算法进行GAZEBO仿真？（三种情况：激光，RGBD，离线地图）
 - 如何进行真实的实验？
  
## 局部路径规划介绍
局部路径规划（局部避障）常见的算法有人工势场、直方图法和动态窗口法。
人工势场法APF是局部路径规划的一种比较常用的方法。这种方法假设机器人在一种虚拟力场下运动。人工势场包括引力场合斥力场，其中目标点对物体产生引力，引导物体朝向其运动。障碍物对物体产生斥力，避免物体与之发生碰撞。物体在路径上每一点所受的合力等于这一点所有斥力和引力的和。这里的关键是如何构建引力场和斥力场。   

APF存在的问题
（a） 当物体离目标点比较远时，引力将变的特别大，相对较小的斥力在甚至可以忽略的情况下，物体路径上可能会碰到障碍物  
（b）当目标点附近有障碍物时，斥力将非常大，引力相对较小，物体很难到达目标点  
（c）在某个点，引力和斥力刚好大小相等，方向想反，则物体容易陷入局部最优解或震荡  

可以通过对引力场和斥力函数，以及添加随机扰动等方式缓解上述问题。

APF参考:  
http://kovan.ceng.metu.edu.tr/~kadir/academia/courses/grad/cs548/hmws/hw2/report/apf.pdf

## APF算法介绍 （原理+订阅的topic+发布的topic+参数）
算法包含	planner_node.cpp， local_planning.cpp， apf.cpp， planning_visualization.cpp  
其中，    
### 1) planner_node.cpp为运行的主节点，节点名字为local_planner_node。  
### 2) local_plannning.cpp为调用的局部算法管理函数，负责接受目标位置、局部地图和当前无人机odom，并调用局部算法apf生成控制指令，并将指令发送出来。    
  接收话题：  
  "/prometheus/planning/odom_world"    
  "/prometheus/planning/local_pcl" ， 局部观测的点云数据，位于局部坐标系，需要通过odom 才能转化到全局map坐标系下面。  
  "/prometheus/planning/goal"  ， 目标是在全局坐标系map下给出的。    
  发布话题：
  "/prometheus/local_planner/desired_vel"  ， 全局坐标系下的期望控制速度    
  "/prometheus/planning/stop_cmd"  ，当前飞行器是否安全     

### 3) apf.cpp为人工势场算法，接受地图、起点和终点，生成速度控制指令。  
  需要在launch文件中设置的参数：  
    nh.param("apf/inflate_distance", inflate_distance, 0.20);  // 感知障碍物距离  
    nh.param("apf/obs_distance", obs_distance, 2.5);  // 感知障碍物距离  
    nh.param("apf/k_push", k_push, 0.8);                         // 推力增益  
    nh.param("apf/k_att", k_att, 0.4);                                  // 引力增益  
    nh.param("apf/min_dist", min_dist, 0.2);                            // 最小壁障距离  
    nh.param("apf/max_att_dist", max_att_dist, 5.0);             // 最大吸引距离  
    nh.param("apf/ground_height", ground_height, 0.1);  // 地面高度  
    nh.param("apf/ground_safe_height", ground_safe_height, 0.2);  // 地面安全距离  
    nh.param("apf/safe_distance", safe_distance, 0.15);  // 安全停止距离  

### 4) palnning_visualization负责可视化轨迹。  
   发布话题为“/planning_vis/trajectory”
  
## Gazebo仿真环境运行  
  
  分三种情况来写，对应不同的指令及rviz设置

 - 运行launch文件  
  `roslaunch prometheus_gazebo sitl_local_planner.launch`  
 - 在rviz中指定目标点  
 - 或通过终端输入目标点  
 - 更多详细信息：演示视频  
  

### 运行截图  

## 真实环境中运行  
  

待补充  
  

