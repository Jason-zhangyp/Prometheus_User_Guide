# 使用FastPlanner进行轨迹优化
  
## 规划框架
FastPlanner 由三部分组成，分别包括混合A*算法、Bspline轨迹生成和自动时间分配调整。规划流程的主要管理由plannning_fsm函数完成，负责调用规划算法，安全检查等。  

混合A*算法位于path_searching文件夹，用于实现全局规划 ，主要调用函数为 "search()"，在已知地图的前提下，给定起点和终点状态(位置和速度)实现混合A*搜索；如果搜索成功，返回一系列path_nodes_节点。

Bspline轨迹位于bspline_opt文件夹，用于轨迹生成。将混合A*的输出轨迹点转化为Bspline的控制点，再调用optimize函数完成优化。

自动时间分配位于位于bspline_opt文件夹，用于对样条轨迹进行时间分配优化（reallocateTime），保证整个轨迹的动力学安全。

另外，系统中还包含一条安全检查线程（safetyCallback），按照固定频率对输入目标点、生成的轨迹进行安全性检查，保证系统随着地图更新也能一直安全。

最后在traj_serve中把得到的bspline轨迹进行处理，得到cmd (prometheus_msgs::PositionReference) 指令，发送给飞控使用。


## FastPlanner算法介绍 （原理+订阅的topic+发布的topic+参数）
主要部分包括：  
 - 主节点： planning_fsm.cpp  
 - 混合A*: kinodynamic_astar.cpp  
 - Bspline优化: bspline_optimizer.cpp
 - 自动时间分配： non_uniform_bspline.cpp
 - sdf地图： sdf_map.cpp     global_point_sdf.cpp
 - 轨迹处理： traj_server.cpp


**fast fsm**  

文件目录： `FastPlanner/plan_manage/src/planning_fsm.cpp `

功能：

1. 管理规划节点，运行planner状态机
2. 安全性检查  
3. 订阅目标，发布规划的轨迹


参数：   

      nh.param("bspline/limit_vel", NonUniformBspline::limit_vel_, -1.0);  // 速度限制  
      nh.param("bspline/limit_acc", NonUniformBspline::limit_acc_, -1.0); // 加速度限制  
      nh.param("bspline/limit_ratio", NonUniformBspline::limit_ratio_, -1.0); // 时间分配的安全约束
      nh.param("fsm/flight_type", flight_type_, -1);  //飞行模式，手动还是自动加载goal  
      nh.param("fsm/thresh_replan", thresh_replan_, -1.0);  //  无人机单次规划起点距离限制  
      nh.param("fsm/thresh_no_replan", thresh_no_replan_, -1.0);  //  重规划终点限制  
      nh.param("fsm/safety_distance", safety_distance, 0.3); //安全距离  
      nh.param("sdf_map/SDF_MODE", sdf_mode, 0); // sdf模式，如果为0,非增量式sdf；为1，fast_planner默认增量式sdf  


订阅话题：

    "/prometheus/planning/goal"


发布话题： 

    "/prometheus/planning/stop_cmd"  // 当前是否安全的状态指令
    "/prometheus/planning/bspline"  //  规划出来的bspline轨迹



**混合A\***  

文件目录： `FastPlanner/path_searching/src/kinodynamic_astar.cpp`

参数：  

       nh.param("search/max_tau", max_tau_, -1.0);  //如果考虑对时间维度进行划分才设置，这里未设置
       nh.param("search/init_max_tau", init_max_tau_, -1.0);  
       nh.param("search/max_vel", max_vel_, -1.0);    // 速度限制
       nh.param("search/max_acc", max_acc_, -1.0);    // 加速度限制
       nh.param("search/w_time", w_time_, -1.0);    //
       nh.param("search/horizon", horizon_, -1.0);   //限制全局规划的距离，保证实时性
       nh.param("search/resolution_astar", resolution_, -1.0);   //空间分辨率
       nh.param("search/time_resolution", time_resolution_, -1.0);   // 时间维度分辨率
       nh.param("search/lambda_heu", lambda_heu_, -1.0);   // 启发函数权重
       nh.param("search/margin", margin_, -1.0);  //检测碰撞
       nh.param("search/allocate_num", allocate_num_, -1);  //最大节点数目
       nh.param("search/check_num", check_num_, -1);  //对中间状态安全检查



**Bspline优化**  

文件目录：`FastPlanner/bspline_opt/src/bspline_optimizer.cpp`

参数：

      nh.param("optimization/lamda1", lamda1_, -1.0);  // 优化时加权参数
      nh.param("optimization/lamda2", lamda2_, -1.0);
      nh.param("optimization/lamda3", lamda3_, -1.0);
      nh.param("optimization/lamda4", lamda4_, -1.0);
      nh.param("optimization/lamda5", lamda5_, -1.0);
      nh.param("optimization/dist0", dist0_, -1.0);
      nh.param("optimization/dist1", dist1_, -1.0);
      nh.param("optimization/max_vel", max_vel_, -1.0);
      nh.param("optimization/max_acc", max_acc_, -1.0);
      nh.param("optimization/max_iteration_num", max_iteration_num_, -1);
      nh.param("optimization/algorithm", algorithm_, -1); // 最好的算法 40: SLSQP, 11:LBFGS
      nh.param("optimization/order", order_, -1); 



**自动时间分配**

文件目录：`FastPlanner/bspline_opt/src/non_uniform_bspline.cpp`


**地图处理**  

文件目录：  

    FastPlanner/plan_env/src/sdf_map.cpp
    FastPlanner/plan_env/src/global_point_sdf.cpp

功能：
    1. 订阅点云数据
    2. 处理点云数据并膨胀地图
    3. 得到sdf地图

参数：  

    node_.param("sdf_map/origin_x", origin_(0), -20.0);  //地图原点
    node_.param("sdf_map/origin_y", origin_(1), -20.0);
    node_.param("sdf_map/origin_z", origin_(2), 0.0);

    node_.param("sdf_map/map_size_x", map_size_(0), 40.0);  //地图大小边界
    node_.param("sdf_map/map_size_y", map_size_(1), 40.0);
    node_.param("sdf_map/map_size_z", map_size_(2), 5.0);

    node_.param("sdf_map/resolution_sdf", resolution_sdf_, 0.2);  //地图分辨率
    node_.param("sdf_map/ceil_height", ceil_height_, 2.0);  // 限高
    node_.param("sdf_map/update_rate", update_rate_, 10.0);  // 地图更新频率
    node_.param("sdf_map/update_range", update_range_, 5.0);  // 单次更新距离
    node_.param("sdf_map/inflate", inflate_, 0.2);  // 地图安全膨胀距离

订阅话题

    "/prometheus/planning/odom_world" :  里程计话题
    "/prometheus/planning/global_pcl": 全局坐标系下，点云数据

发布话题

    "/sdf_map/inflate_cloud": 用于可视化的膨胀点云



**样条轨迹处理&发布控制**    

文件目录：`FastPlanner/plan_manage/src/traj_server.cpp`

订阅话题：  

    "/prometheus/planning/bspline" :  由规划器生成的样条轨迹  
    "/prometheus/fast_planning/replan": 重规划标志  
    "/prometheus/planning/odom_world": 里程计数据  

发布话题：  

    "/prometheus/planning/position_cmd": 给无人机控制指令
    "/prometheus/planning/traj":  显示轨迹
    "/prometheus/planning/state":   显示运动



## 规划算法任务程序
**planning_mission.cpp**

请查看局部规划算法文档中的介绍


## Gazebo仿真环境运行  
  
**使用激光雷达作为传感器**
 - 运行launch文件（请查看launch文件参数说明，并进行调整）
  		roslaunch prometheus_gazebo sitl_fast_planning_3dlidar.launch 
 - 在打开的rviz窗口中勾选`Fast_Planner`、`Octomap_Mapping`及`Ground_Truth`显示
 - 输入3选择Fast Planner算法，无人机将自动起飞
 - 在rviz中通过3D Nav Goal按钮指定目标点，点选该按钮后，同时按住鼠标左右键在rviz窗口中选择一点向上拉
    [![G66Zi6.png](https://s1.ax1x.com/2020/04/07/G66Zi6.png)](https://imgchr.com/i/G66Zi6)
 - 红色线代表规划的路径,黄色线代表优化后的轨迹
 - 也可以通过终端发布目标点  
 		rostopic pub /prometheus/planning/goal ...
 - 通过终端查看算法相关信息
   [![G6cw9K.md.png](https://s1.ax1x.com/2020/04/07/G6cw9K.md.png)](https://imgchr.com/i/G6cw9K)
  
  运行截图
  
  [![GOCplR.md.png](https://s1.ax1x.com/2020/04/12/GOCplR.md.png)](https://imgchr.com/i/GOCplR)
  
**使用RGBD相机作为传感器**
 - 运行launch文件（请查看launch文件参数说明，并进行调整）
  		roslaunch prometheus_gazebo sitl_fast_planning_rgbd.launch 
 - 在打开的rviz窗口中勾选`Fast_Planner`,`RTAB建图`及`Ground_Truth`显示
 - 其他同上
 
 运行截图

	[![GOCvHf.md.png](https://s1.ax1x.com/2020/04/12/GOCvHf.md.png)](https://imgchr.com/i/GOCvHf)
## 真实环境中运行  
  

