# Gazebo仿真环境安装及测试

更多Gazebo仿真及代码介绍请参阅文档：**Gazebo仿真模块**



## PX4编译环境及固件代码安装

Prometheus项目中的Gazebo仿真模块依赖PX4固件及sitl_gazebo包，因此需先安装PX4编译环境及固件代码，并完成编译：

- **PX4编译环境安装**： [PX4手册 - getting_started](https://dev.px4.io/v1.10/en/setup/getting_started.html)

     - 建议使用ubuntu.sh脚本进行安装
     - PS：现在gcc工具链可以直接通过  sudo apt install gcc-arm-none-eabi 安装！！

- **下载PX4固件代码**，此处建议使用阿木实验室的Prometheus项目专用的PX4仓库：[Firmware_v110](https://github.com/amov-lab/Firmware_v110)，安装方法如下

     ```
     git clone https://github.com/amov-lab/Firmware_v110
     cd Firmware_v110
     git submodule update --init --recursive
     make px4_sitl gazebo
     ```

**说明**：

- 若使用官方PX4仓库，Prometheus部分功能会失效，需要修改后方能使用（暂无详细说明，需自行解决）
- 此处安装成功的标志为：PX4固件能够编译，并能运行其自带的Gazebo仿真，即运行`make px4_sitl gazebo`能够正常运行Gazebo仿真
- 对PX4固件代码进行任何修改或者执行过`git pull`都需要重新运行`make px4_sitl gazebo`
- github下载缓慢，加速方法：https://blog.csdn.net/qq_44621510/article/details/95251993

**相关链接**：

- PX4 Github主页：[PX4 Github](https://github.com/PX4/Firmware) （代码有分支，请查看v1.10.0分支）
- PX4开发者手册：[PX4手册 v1.10.0](https://dev.px4.io/v1.10/en/) （手册有分支，请查看v1.10.0分支）
- PX4官方仿真教程 ： [PX4手册 - gazebo simulation](https://dev.px4.io/v1.10/en/simulation/gazebo.html)



## 环境变量配置

- 打开终端，并输入如下指令打开`bashrc`文件

    ```
    sudo gedit ~/.bashrc 
    ```
    
- 在打开的文件中手动添加如下指令（以下若存在已添加过的命令，请勿重复添加），其中`${your prometheus path}`为Prometheus项目路径，`${your px4 path}`为安装PX4固件的路径。

    ```c
    source ${your prometheus path}/Prometheus/devel/setup.bash
    export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:${your prometheus path}/Prometheus/devel/lib
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${your prometheus path}/Prometheus/Simulator/gazebo_simulator/models
    source ${your px4 path}/Firmware_v110/Tools/setup_gazebo.bash ${your px4 path}/Firmware_v110 ${your px4 path}/Firmware_v110/build/px4_sitl_default
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${your px4 path}/Firmware_v110
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${your px4 path}/Firmware_v110/Tools/sitl_gazebo
    ```

**备注**：

- 添加以上环境变量后，每次打开终端会出现配置好的路径，忽略即可。
- 此步骤经常容易出错，请再三检查。（每一个路径都是有实际含义，请确保电脑中有该路径存在）
- 提供如下截图用作对比参考
	<img width="600" src="https://s1.ax1x.com/2020/04/12/GOT5IP.md.png"/>



## 仿真用插件安装

- **安装3Dlidar插件**

    ```
    sudo apt-get install ros-melodic-velodyne-gazebo-plugins
    ```

 - **安装octomap在rviz中的插件**
   
    ```
    sudo apt-get install ros-melodic-octomap-rviz-plugins
    ```



## Prometheus仿真功能包编译

 请确保Prometheus项目中其他相应的功能包均已编译通过，然后编译prometheus_gazebo功能包
```
cd Prometheus
./compile_gazebo_simulator.sh
```



## Gazebo仿真运行测试

运行如下命令测试是否Gazebo仿真是否正确配置
```
roslaunch prometheus_gazebo sitl.launch
```
此时，第一个终端同时运行了PX4仿真、Mavros、px4_pos_estimator、px4_pos_controller四个节点，第二个终端则运行了ground_station节点。因此，若第一个终端无报错，第二个终端显示`[Connected]`并能够查看到飞机状态，且Gazebo成功运行，代表**成功运行**。



在Gazebo仿真环境中，根据仿真目的不同提供以下三种方式控制无人机飞行：



#### 1. QGC+遥控器控制（针对不需要offboard模式的情况）	 

 - 软件需求：安装QGC地面站
 - 硬件需求：可连接至电脑的遥控器或手柄，连接后在QGC中进行配置
 - 打开QGC，可通过虚拟摇杆或者外接遥控器对飞机进行操控，**运行截图**如下：
	<img width="600" src="https://s2.ax1x.com/2020/02/25/3tSdk6.png"/>


####  2. 键盘控制
 运行键盘控制节点，根据终端提示进行键盘操纵
 ```
rosrun prometheus_gazebo keyboard_control_px4.py
 ```
需在QGC中配置如下参数（似乎有点问题，待测试）
```
RC2_TRIM = 1000us
COM_FLTMODE1 = Position
RC_CHAN_CNT = 8
RC_MAP_FLTMODE = Channel 5
RC_MAP_PITCH = Channel 3
RC_MAP_ROLL= Channel 1
RC_MAP_THROTTLE = Channel 2
RC_MAP_YAW = Channel 4
```

#### 3. 终端控制（全自主的offboard控制）

运行如下指令启动终端控制节点，并根据终端提示输入指令

```
rosrun prometheus_control terminal_control
```

一般情况下，首先输入999解锁并切换至offboard模式，然后输入1起飞，然后再输入其他指令（悬停、降落、机体系移动、惯性系移动、轨迹追踪等等）

运行截图如下
<img width="600" src="https://s2.ax1x.com/2020/02/25/3tS3pF.png"/>




