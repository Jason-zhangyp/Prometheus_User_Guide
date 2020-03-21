# 如何使用Octomap进行建图

本教程的目的：  

 - 让用户理解什么是建图？
 - Octomap包的使用
 - 如何使用Octomap算法进行GAZEBO仿真？（三种情况：激光，RGBD，离线地图）
 - 如何进行真实的实验？


## RtabMap包的使用（原理+订阅的topic+发布的topic+参数）

主要是对不同情况，给出不同的launch，然后介绍参数，具体可查官网

## Gazebo仿真环境运行

数据集建图demo（使用官网的提供的数据集（备注下载链接，如何跑数据集跑出来建图的效果）

手持建图（在gazebo中使用我们自己的worlds，手动飞行建图的流程，刚好把obstacle那个建一个完整的图）

 - 运行launch文件
	 `roslaunch prometheus_gazebo sitl_mapping.launch`
 - 测试此功能时，无需自主飞行，此处推荐键盘控制或通过遥控器控制无人机 移动
 - 在rviz可观察建图情况
 - 更多详细信息：演示视频

### 运行截图



## 真实环境中运行

待补充
