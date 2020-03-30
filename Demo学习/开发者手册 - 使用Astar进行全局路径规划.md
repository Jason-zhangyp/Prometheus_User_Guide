# 使用Astar进行全局路径规划  
  

本教程的目的：  

 - 让用户理解什么是全局路径规划？什么是Astar算法？
 - Astar算法的原理解释，相关参数解释
 - 如何使用Astar算法进行GAZEBO仿真？（三种情况：激光，RGBD，离线地图）
 - 如何进行真实的实验？
  
## 全局路径规划介绍

#### 1) 全局算法和局部算法 
> 全局路径规划是在已知的环境中，给机器人规划一条路径，路径规划的精度取决于环境获取的准确度，全局路径规划可以找到最优解，但是需要预先知道环境的准确信息，当环境发生变化，如出现未知障碍物时，该方法就无能为力了。它是一种事前规划，因此对机器人系统的实时计算能力要求不高，虽然规划结果是全局的、较优的，但是对环境模型的错误及噪声鲁棒性差。   

> 局部路径规划则环境信息完全未知或有部分可知，侧重于考虑机器人当前的局部环境信息，让机器人具有良好的避障能力，通过传感器对机器人的工作环境进行探测，以获取障碍物的位置和几何性质等信息，这种规划需要搜集环境数据，并且对该环境模型的动态更新能够随时进行校正，局部规划方法将对环境的建模与搜索融为一体，要求机器人系统具有高速的信息处理能力和计算能力，对环境误差和噪声有较高的鲁棒性，能对规划结果进行实时反馈和校正，但是由于缺乏全局环境信息，所以规划结果有可能不是最优的，甚至可能找不到正确路径或完整路径。 

> 两者协同工作，机器人可更好的规划从起始点到终点的行走路径。   

#### 2) A*算法  
>A*算法是全局启发式搜索算法，是一种尽可能基于现有信息的搜索策略，也就是说搜索过程中尽量利用目前已知的诸如迭代步数，以及从初始状态和当前状态到目标状态估计所需的费用等信息。     

>A*算法在选择下一个被检查的节点时考虑来之前花费代价，同时使用到目标距离作为引导，两者之和作为评价该节点处于最优路线上的可能性的量度，这样可以首先搜索到位于最优路径上可能性大的节点。     

>A*算法的基本思想如下：引入当前节点j的估计函数$f^*$,当前节点j的估计函数定义为：    

>$f^*(j)= g(j)+h^*(j)$     

>其中$g(j)$是从起点到当前节点j的实际代价的量度，$h^*(j)$是从节点j到终点的最小消耗的估计，可以依据实际情况，选择$h^*(j)$的具体形式，$h^*(j)$要满足一个要求：不能高于节点j到终点的实际最小费用。从起始节点点向目的节点搜索时，每次都搜索$f^*(j)$最小的节点，直到发现目的节点。   



## Astar算法介绍 （原理+订阅的topic+发布的topic+参数）

#### global_planner_node.cpp
功能：  
  生成主要node. 

#### global_planning.cpp  
功能：  
  全局规划的管理函数
设置参数：   
  nh.param("planning/safe_distance", safe_distance, 0.25)， 停止距离
  接受topic:   
  "/prometheus/planning/goal"， 目标
  "/prometheus/planning/odom_world"， map坐标系下位资
  "/prometheus/planning/global_pcl"， map坐标系下全局点云
  发布topic: 
  "/prometheus/planning/path_cmd"， A*生成的轨迹指令
  "/prometheus/planning/stop_cmd"， 当前是否安全的状态指令
  "/prometheus/planning/global_map_marker"， 显示地图

#### A_star.cpp  
功能：
A*的算法实现
参数：
  nh.param("astar/resolution_astar", resolution_, 0.2);
  nh.param("astar/lambda_heu", lambda_heu_, 2.0);
  nh.param("astar/allocate_num", allocate_num_, 1000);

#### occupy_map.cpp   
功能：  
  1） 接受全局地图，进行膨胀处理。
  2） 查询对应点在膨胀地图的状态。

设置参数：    
    nh.param("astar/resolution_astar", resolution_,  0.2);  地图分辨率   
    nh.param("astar/inflate", inflate_,  0.3);  膨胀距离    
    nh.param("map/map_size_x", map_size_3d_(0), 10.0);   地图大小    
    nh.param("map/map_size_y", map_size_3d_(1), 10.0);  地图大小    
    nh.param("map/map_size_z", map_size_3d_(2), 5.0);   地图高度    
    nh.param("map/origin_x", origin_(0), -5.0);   地图起点   
    nh.param("map/origin_y", origin_(1), -5.0);     
    nh.param("map/origin_z", origin_(2), 0.0);    
    nh.param("map/ceil_height_", ceil_height_, 4.9);  地图限高    
    nh.param("map/floor_height_", floor_height_, 0.1); 地图限最低高     

#### planning_visualization.cpp   
  功能：  
  显示轨迹   
  发布topic:   
  “/planning_vis/trajectory”    
## Gazebo仿真环境运行  
  
  分三种情况来写（三种情况：激光，RGBD，离线地图），对应不同的指令及rviz设置    
  怎么给goal，   

 - 运行launch文件  
  `roslaunch prometheus_gazebo sitl_local_planner.launch`  
 - 在rviz中指定目标点  
 - 或通过终端输入目标点  
 - 更多详细信息：演示视频  
  

### 运行截图  

## 真实环境中运行  
  

待补充  
  

