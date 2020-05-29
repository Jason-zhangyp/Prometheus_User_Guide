# 使用VFH进行局部路径规划
  
## 局部路径规划
局部路径规划（局部避障）常见的算法有人工势场（APF）、直方图法（VFH）和动态窗口法（DWA)。

   VFH 算法主要是将无人机周围的环境转换成向量场直方图 , 根据极坐标直方图筛选出下一步可以通行的方向 , 并选取一个最优方向以一定的步长 行驶进入下一位置 。 当进入下一个位置后继续探测 , 直到达到目标点位置 。
   VFH 主要包含3个步骤：  
   向量场直方图的建立  
   选定可通行扇区（或者扇区打分）  
   确定最优通行方向  


## VFH算法程序介绍
与算法相关的程序有	
 - 主节点：`planner_node.cpp`
 - 局部规划算法管理函数：`local_planning.cpp`
 - 人工势场算法： `vfh.cpp`
 - 轨迹可视化节点：`planning_visualization.cpp `

**local_plannning.cpp**

订阅目标位置、局部地图和当前无人机odom，并调用局部算法程序(例如：`apf.cpp`， 'vfh.cpp'，通过 "planning/algorithm_mode"来控制)生成控制指令，并发布该指令    
nh.param("planning/algorithm_mode", algorithm_mode, 0);  //用于设置运行算法的模式，apf=0(默认)，vfh=1
订阅话题：  
- "/prometheus/planning/odom_world"    
- "/prometheus/planning/local_pcl"，局部观测的点云数据，位于局部坐标系，需要通过odom 才能转化到全局map坐标系下面
-  "/prometheus/planning/goal"，目标点坐标，目标点是在全局坐标系map下给出的 

发布话题：
 - "/prometheus/planning/desired_vel"  ， 全局坐标系下的期望速度    
 - "/prometheus/planning/stop_cmd"  ，当前飞行器是否安全     

**vfh.cpp**

接收地图、起点和终点，并生成速度控制指令。  
参数说明（可在launch文件中设置）
    nh.param("vfh/inflate_distance", inflate_distance, 0.20);  // 感知障碍物距离  
    nh.param("vfh/obs_distance", obs_distance, 3.0);  // 感知障碍物距离  
    nh.param("vfh/max_att_dist", max_att_dist, 5.0);             // 最大吸引距离  
    nh.param("vfh/safe_distance", safe_distance, 0.2); // 安全停止距离  
    nh.param("vfh/goalWeight", goalWeight, 0.2); // 目标权重  
    nh.param("vfh/prevWeight", prevWeight, 0.0); // 光滑权重  
    nh.param("vfh/obstacle_weight", obstacle_weight, 0.0); // 障碍物权重  
    nh.param("vfh/limit_v_norm", limit_v_norm, 0.4); // 极限速度  

**palnning_visualization.cpp**

发布话题为“/planning_vis/trajectory”
 
## 局部规划算法任务程序
**planning_mission.cpp**

该程序负责执行所有规划算法的任务：订阅规划算法发布的指令，处理后发送给底层控制程序。

程序运行逻辑：
- 选择规划算法，选择后飞机将自动解锁并起飞
		cout << "Please choose the planning method: 1 for APF, 2 for A*, 3 for Fast planner"<<endl;
- 初始状态为等待规划算法发布指令（若不给定目标点，规划算法不发布）
		cout << "Waiting for trajectory" << endl;
- 当接收到指令后，则根据不同规划算法生成底层控制指令并发布，具体流程请查看源码
   - `APF_planner();`
   - `A_star_planner();`
   - `Fast_planner();`
- 若抵达目标点附近，则无人机悬停
		if (distance_to_goal < MIN_DIS)
- 若接收到紧急停止指令，则无人机原地悬停
        cout << "Dangerous! Hold there." << endl; 
        
参数：
 - 是否控制偏航角：若设为true，偏航角将根据规划出来的指令进行平滑更新，否则航向将打死为0度
		nh.param<bool>("planning_mission/control_yaw_flag", control_yaw_flag, true);

主要订阅话题：
 - 全局规划算法发布的路径指令 
 		ros::Subscriber global_planner_sub = nh.subscribe<nav_msgs::Path>("/prometheus/planning/a_star/desired_path", 50, global_planner_cmd_cb);
 - 局部规划算法发布的速度指令
		ros::Subscriber local_planner_sub  =    nh.subscribe<geometry_msgs::Point>("/prometheus/planning/apf/desired_vel", 50, local_planner_cmd_cb);
 - FastPlanner算法发布的控制指令（期望轨迹）
  		ros::Subscriber fast_planner_sub   =    nh.subscribe<prometheus_msgs::PositionReference>("/prometheus/planning/fastplanner/desired_trajecotry", 50, fast_planner_cmd_cb);
 -  紧急停止指令
 		ros::Subscriber stop_cmd_sub = nh.subscribe<std_msgs::Int8>("/prometheus/planning/stop_cmd", 10, stop_cmd_cb);  

发布话题：
 - 控制指令，发送给控制模块 [px4_pos_controller.cpp]的命令
		command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

## Gazebo仿真环境运行  
  
  **使用激光雷达作为传感器**
 - 运行launch文件（请查看launch文件参数说明，并进行调整）
		roslaunch prometheus_gazebo sitl_local_planning_3Dlidar_vfh.launch
 - 在打开的rviz窗口中勾选`局部规划`及`Ground_Truth`显示(绿色为真值,红色为激光雷达局部点云)
 - 输入1选择APF算法，无人机将自动起飞(这里使用APF来打印，但是算法已经切换到vfh)
 - 在rviz中通过3D Nav Goal按钮指定目标点，点选该按钮后，同时按住鼠标左右键在rviz窗口中选择一点向上拉 ,然后释放
    [![G66Zi6.png](https://s1.ax1x.com/2020/04/07/G66Zi6.png)](https://imgchr.com/i/G66Zi6)
 - 也可以通过终端发布目标点  
 		rostopic pub /prometheus/planning/goal ...
 - 通过终端查看算法相关信息
   [![G66EIx.md.png](https://s1.ax1x.com/2020/04/07/G66EIx.md.png)](https://imgchr.com/i/G66EIx)
 
运行截图
  待补充

## 真实环境中运行  
  

待补充  
  

