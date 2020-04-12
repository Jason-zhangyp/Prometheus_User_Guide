# Gazebo仿真模块 

关于Gazebo仿真环境安装，请参阅文档：**Gazebo仿真环境安装及测试**



## 功能包简介

**prometheus_gazebo**为Prometheus项目的仿真模块，可为项目的绝大部分功能提供基于Gazebo的仿真测试功能。目前，模块内包含了自定义的Gazebo模型、Gazebo世界、Gazebo插件、仿真测试专用的程序及相关配置文件、启动脚本等内容，具体代码结构说明如下表。

| 目录 | 功能 |
|--|--|
| gazebo_simulator/config|  mavros配置文件、rviz配置文件 |
| gazebo_simulator/gazebo_plugin|  功能性插件  |
| gazebo_simulator/include|   头文件  |
| gazebo_simulator/launch| 启动脚本 |
| gazebo_simulator/maps| 离线地图 |
| gazebo_simulator/models |  自定义Gazebo模型 |
| gazebo_simulator/worlds |  自定义Gazebo世界 |
| gazebo_simulator/cpp_nodes | 专用于仿真的cpp节点	|
| gazebo_simulator/py_nodes | 专用于仿真的python节点 |
| gazebo_simulator/CMakeLists.txt| 功能包编译脚本 |

具体的仿真功能实现请参看各功能说明文档，此文档主要介绍Gazebo仿真功能。

## launch启动脚本

大部分脚本均构建于sitl.launch之上，因此此处只介绍sitl.launch。一共可分为四个部分，启动PX4中的SITL功能、启动Gazebo仿真器、启动Mavros及启动Prometheus相关代码。如果想深入了解Gazebo仿真背后的具体实现细节，弄清楚PX4 - Mavros - Prometheus - Gazebo这四者的具体启动内容和相互之间的关系即可。

```shell
<launch>
    <!-- 启动PX4中的SITL功能 -->
    <!-- vehicle model and world -->
    <env name="PX4_SIM_MODEL" value="solo" />
    <env name="PX4_ESTIMATOR" value="ekf2_gps" />

    <!-- PX4 configs -->
    <arg name="interactive" default="true"/>
    <!-- PX4 SITL -->
    <arg unless="$(arg interactive)" name="px4_command_arg1" value="-d"/>
    <arg     if="$(arg interactive)" name="px4_command_arg1" value=""/>
    <!-- <node name="sitl" pkg="px4" type="px4" output="screen" args="$(find px4)/ROMFS/px4fmu_common -s etc/init.d-posix/rcS $(arg px4_command_arg1)" required="true"/> -->
    <node name="sitl" pkg="px4" type="px4" output="screen" args="$(find px4)/ROMFS/px4fmu_common -s etc/init.d-posix/rcS $(arg px4_command_arg1)"/>

    <!-- 启动Gazebo -->
    <!-- Gazebo configs -->
    <arg name="gui" default="true"/>
    <arg name="world" default="$(find prometheus_gazebo)/worlds/empty.world"/>
    <!-- Gazebo sim -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="gui" value="$(arg gui)"/>
        <arg name="world_name" value="$(arg world)"/>
    </include>

    <!-- Spawn vehicle model -->
    <!-- https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_ros/scripts/spawn_model -->
    <arg name="x" default="0"/>
    <arg name="y" default="0"/>
    <arg name="z" default="0.1"/>
    <arg name="R" default="0"/>
    <arg name="P" default="0"/>
    <arg name="Y" default="0"/>
    <arg name="sdf" default="$(find prometheus_gazebo)/models/P300_basic/P300_basic.sdf"/>
    <arg name="model" default="P300_basic"/>
    <node name="$(anon vehicle_spawn)" pkg="gazebo_ros" type="spawn_model" output="screen" 
        args="-sdf -file $(arg sdf) -model $(arg model) -x $(arg x) -y $(arg y) -z $(arg z) -R $(arg R) -P $(arg P) -Y $(arg Y)">
    </node>

    <!-- 启动MAVROS -->
    <node pkg="mavros" type="mavros_node" name="mavros" output="screen">
        <param name="fcu_url" value="udp://:14540@localhost:14557" />
        <param name="gcs_url" value="" />
        <param name="target_system_id" value="1" />
        <param name="target_component_id" value="1" />
        <rosparam command="load" file="$(find prometheus_gazebo)/config/px4_pluginlists.yaml" />
        <rosparam command="load" file="$(find prometheus_gazebo)/config/px4_config.yaml" />
    </node>

    <!-- TF transform -->
    <include file="$(find prometheus_gazebo)/launch/tf_transform.launch">
        <arg name="x" value="$(arg x)"/>
        <arg name="y" value="$(arg y)"/>
        <arg name="z" value="$(arg z)"/>
    </include>

    <!-- 启动Prometheus代码 -->
    <!-- run the px4_pos_estimator.cpp -->
    <node pkg="prometheus_control" type="px4_pos_estimator" name="px4_pos_estimator" output="screen">
        <rosparam command="load" file="$(find prometheus_control)/launch/Parameter_for_control.yaml" />
    </node>

    <!-- run the px4_pos_controller.cpp -->
    <node pkg="prometheus_control" type="px4_pos_controller" name="px4_pos_controller" output="screen">
        <rosparam command="load" file="$(find prometheus_control)/launch/Parameter_for_control.yaml" />
    </node>

    <!-- run the ground_station.cpp -->
    <node pkg="prometheus_control" type="ground_station" name="ground_station" output="screen" launch-prefix="gnome-terminal --tab --">	
    </node>
</launch>
```



**PX4**

PX4是一个功能丰富的开源飞控，它所提供的不仅仅只是飞控代码，另外一个重要的工具就是飞行仿真。PX4提供的软件在环仿真（SITL）工具一共是有[jMAVSim](https://dev.px4.io/en/simulation/jmavsim.html)、[Gazebo](https://dev.px4.io/en/simulation/gazebo.html)、[AirSim](https://dev.px4.io/en/simulation/airsim.html)这三种。Gazebo仿真是PX4官方高度推荐的仿真器，支持旋翼、固定翼、倾转、小车等，是所有仿真器里支持平台最多的，也能支持多个无人机的仿真。





## Gazebo模型
目前所包含的模型大致分为下面几类，传感器模型、飞机模型、道具模型等。

#### 传感器模型
 - 2Dlidar（默认参数：角分辨率1度,探测范围0.1-10米,发布话题为/prometheus/sensors/2Dlidar等等等）
 - 3Dlidar（默认参数：角分辨率1度,探测范围0.1-10米,16线,发布话题为/prometheus/sensors/3Dlidar 等等等, 利用了libgazebo_ros_velodyne_laser.so）
 - D435i (参数待调整，利用了librealsense_gaezbo_plugin.so)
 - Monocular（单目，利用了libgazebo_ros_camera.so）

#### 飞机模型
 - P300_basic (根据solo无人机修改而成，后期需要替换)
 - P300_2Dlidar(P300_basic + 2Dlidar)
 - P300_3Dlidar(P300_basic + 3Dlidar)
 - P300_D435i(P300_basic + D435i)
 - P300_Monocular_down(P300_basic + Monocular朝下)
 - P300_Monocular_front(P300_basic + Monocular超前)

#### 道具模型
- car_landing_pad（降落用的吉普车）
- circle_gate（形状穿越用的圆形门）
- wall_with_num（数字识别用的数字墙）
- line_following（巡线用的线）

## Gazebo仿真中的坐标系说明
有如下坐标系（括号内代表该坐标系的frame_id）：
- **世界坐标系（world）**：原点位于Gazebo仿真器中心点，xyz轴方向遵从ENU的右手法则
- **机体系（base_link）**：原点位于无人机质心，x轴指向机体前方，y轴指向机体左方，z轴指向机体上方
- **起飞坐标系（world_on_body（目前名字为map））**：原点位于无人机的初始点
- **传感器坐标系（3Dlidar_link、realsense_camera_link等）**：原点位于传感器质心
- **相机坐标系**：原点位于相机光心，从相机往前看，物体在相机右方x为正，下方y为正，前方z为正；因此图像识别得到的结果应当转换至机体系，该部分处理在对应的程序中进行

仿真中与坐标系相关的话题：
- p300_basic.sdf中插件所发布的里程计真值（`/prometheus/ground_truth/p300_basic`）位于world坐标系
- 3Dlidar.sdf及D435i.sdf等传感器发布的测量数据（如点云）均位于传感器坐标系，因此需要转换至机体系方可使用，该部分处理在`tf_transform.launch`中
- px4_pos_estimator.cpp中订阅`/prometheus/ground_truth/p300_basic`，并转存为`/mavros/vision_pose/pose`发布
- PX4固件通过Mavros功能包发布的位置、速度、里程计等信息位于起飞坐标系（world_on_body（目前名字为map））中
- rtab_map和octomap等建图工具包发布的地图或点云信息，位于世界坐标系（world）
