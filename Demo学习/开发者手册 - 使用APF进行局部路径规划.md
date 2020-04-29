# 使用APF进行局部路径规划
  
## 局部路径规划
局部路径规划（局部避障）常见的算法有人工势场、直方图法和动态窗口法。

人工势场法APF是局部路径规划的一种比较常用的方法。这种方法假设机器人在一种虚拟力场下运动。人工势场包括引力场合斥力场，其中目标点对物体产生引力，引导物体朝向其运动。障碍物对物体产生斥力，避免物体与之发生碰撞。物体在路径上每一点所受的合力等于这一点所有斥力和引力的和。这里的关键是如何构建引力场和斥力场。   

APF存在的问题
- 当物体离目标点比较远时，引力将变的特别大，相对较小的斥力在甚至可以忽略的情况下，物体路径上可能会碰到障碍物  
- 当目标点附近有障碍物时，斥力将非常大，引力相对较小，物体很难到达目标点  
- 在某个点，引力和斥力刚好大小相等，方向想反，则物体容易陷入局部最优解或震荡  

可以通过对引力场和斥力函数，以及添加随机扰动等方式缓解上述问题。

APF参考阅读: http://kovan.ceng.metu.edu.tr/~kadir/academia/courses/grad/cs548/hmws/hw2/report/apf.pdf

## APF算法程序介绍
与算法相关的程序有	
 - 主节点：`planner_node.cpp`
 - 局部规划算法管理函数：`local_planning.cpp`
 - 人工势场算法：`apf.cpp`
 - 轨迹可视化节点：`planning_visualization.cpp `

**local_plannning.cpp**

订阅目标位置、局部地图和当前无人机odom，并调用局部算法程序`apf.cpp`生成控制指令，并发布该指令    
  
订阅话题：  
- "/prometheus/planning/odom_world"    
- "/prometheus/planning/local_pcl"，局部观测的点云数据，位于局部坐标系，需要通过odom 才能转化到全局map坐标系下面
-  "/prometheus/planning/goal"，目标点坐标，目标点是在全局坐标系map下给出的 

发布话题：
 - "/prometheus/planning/apf/desired_vel"  ， 全局坐标系下的期望速度    
 - "/prometheus/planning/stop_cmd"  ，当前飞行器是否安全     

**apf.cpp**

接收地图、起点和终点，并生成速度控制指令。  
参数说明（可在launch文件中设置）
 - nh.param("apf/inflate_distance", inflate_distance, 0.20);  // 感知障碍物距离  
 - nh.param("apf/obs_distance", obs_distance, 2.5);  // 感知障碍物距离  
 - nh.param("apf/k_push", k_push, 0.8);                         // 推力增益  
 - nh.param("apf/k_att", k_att, 0.4);                                  // 引力增益  
 - nh.param("apf/min_dist", min_dist, 0.2);                            // 最小壁障距离  
 - nh.param("apf/max_att_dist", max_att_dist, 5.0);             // 最大吸引距离  
 - nh.param("apf/ground_height", ground_height, 0.1);  // 地面高度  
 - nh.param("apf/ground_safe_height", ground_safe_height, 0.2);  // 地面安全距离  
 - nh.param("apf/safe_distance", safe_distance, 0.15);  // 安全停止距离  

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
  		roslaunch prometheus_gazebo sitl_local_planning_3Dlidar.launch 
 - 在打开的rviz窗口中勾选`局部规划`及`Ground_Truth`显示(绿色为真值,红色为激光雷达局部点云)
 - 输入1选择APF算法，无人机将自动起飞
 - 在rviz中通过3D Nav Goal按钮指定目标点，点选该按钮后，同时按住鼠标左右键在rviz窗口中选择一点向上拉  
    [![G66Zi6.png](https://s1.ax1x.com/2020/04/07/G66Zi6.png)](https://imgchr.com/i/G66Zi6)
 - 也可以通过终端发布目标点  
 		rostopic pub /prometheus/planning/goal ...
 - 通过终端查看算法相关信息
   [![G66EIx.md.png](https://s1.ax1x.com/2020/04/07/G66EIx.md.png)](https://imgchr.com/i/G66EIx)
 
运行截图
 
 ![GLvcpq.gif](https://s1.ax1x.com/2020/04/12/GLvcpq.gif)
  **使用RGBD相机作为传感器**
  

 - 运行launch文件（请查看launch文件参数说明，并进行调整）
  		roslaunch prometheus_gazebo sitl_local_planning_rgbd.launch 
 - 其他同上
  
  说明:  并不推荐此方案,因为RGBD相机视场较差,避障效果较差,如需使用此方案,需对参数进行调试

## 真实环境中运行  
  

待补充  
  

