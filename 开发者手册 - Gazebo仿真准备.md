# Gazebo仿真准备

## PX4固件安装

Gazebo仿真环境依赖PX4固件及sitl_gazebo包，因此需先安装PX4编译环境及代码，并通过编译。
 - [PX4 Github](https://github.com/PX4/Firmware) （请使用v1.10.0分支；旧分支略有区别，需修改启动代码）
 - [PX4手册 v1.10.0](https://dev.px4.io/v1.10/en/) （手册也有分支，请查看v1.10.0分支）
	 - 编译环境安装 ： [PX4手册 - getting_started](https://dev.px4.io/v1.10/en/setup/getting_started.html)
	 - Gazebo仿真教程 ： [PX4手册 - gazebo simulation](https://dev.px4.io/v1.10/en/simulation/gazebo.html)
 - 	建议使用阿木实验室的专用PX4仓库：[Firmware_v110](https://github.com/amov-lab/Firmware_v110)
 -  若使用官方PX4仓库，Prometheus部分功能会失效，需要修改后方能使用
 ```
 git clone https://github.com/amov-lab/Firmware_v110
 cd Firmware_v110
 git submodule update --init --recursive
 make px4_sitl gazebo
```
 - **要求**：PX4固件能够编译，并能运行其自带的Gazebo仿真

## 环境变量配置
打开`bashrc`文件

```
sudo gedit ~/.bashrc 
```
在打开的文件中手动添加（以下若已添加过的命令，请勿重复添加）
```
source ${your prometheus path}/Prometheus/devel/setup.bash
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:${your prometheus path}/Prometheus/devel/lib
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${your prometheus path}/Prometheus/Simulator/gazebo_simulator/models
source ${your px4 path}/Firmware_v110/Tools/setup_gazebo.bash ${your px4 path}/Firmware_v110 ${your px4 path}/Firmware_v110/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${your px4 path}/Firmware_v110
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${your px4 path}/Firmware_v110/Tools/sitl_gazebo
```
其中`${your prometheus path}`为Prometheus项目路径，`${your px4 path}`为安装PX4固件的路径。

添加以上环境变量后，每次打开终端会出现配置好的路径，忽略即可。


## 安装3Dlidar插件

```
sudo apt-get install ros-melodic-velodyne-gazebo-plugins
```

## 仿真功能包编译
 
 请确保Prometheus项目中其他相应的功能包均已编译通过，然后编译prometheus_gazebo功能包
```
cd Prometheus
./compile_gazebo_simulator.sh
```


## 运行测试

运行如下命令测试是否Gazebo仿真是否正确配置
```
roslaunch prometheus_gazebo sitl.launch
```
此时，第一个终端同时运行了PX4仿真、Mavros、px4_pos_estimator、px4_pos_controller四个节点，第二个终端则运行了ground_station节点。

因此，若第一个终端无报错，第二个终端显示[Connected]并能够查看到飞机状态，且Gazebo成功运行，代表成功运行。

在Gazebo仿真环境中，根据仿真目的不同提供以下三种方式控制无人机飞行：

### QGC及遥控器控制（针对不需要offboard模式的情况）	 
 
 - 软件需求：安装QGC地面站
 - 硬件需求：可连接至电脑的遥控器或手柄，连接后在QGC中进行配置
 - 打开QGC，可通过虚拟摇杆或者外接遥控器对飞机进行操控
 
 运行截图如下：
	 
[![3tSdk6.png](https://s2.ax1x.com/2020/02/25/3tSdk6.png)](https://imgchr.com/i/3tSdk6)
	 
###  键盘控制
 运行键盘控制节点，根据终端提示进行键盘操纵
 ```
rosrun prometheus_gazebo keyboard_control)px4.py
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

### 终端控制（全自主的offboard控制）

运行终端控制节点，并根据终端提示输入指令

```
rosrun prometheus_control terminal_control
```

一般情况下，是先输入999解锁并切换至offboard模式，然后输入1起飞，然后再输入其他指令或启动其他全自主任务节点。

运行截图如下

[![3tS3pF.png](https://s2.ax1x.com/2020/02/25/3tS3pF.png)](https://imgchr.com/i/3tS3pF)



