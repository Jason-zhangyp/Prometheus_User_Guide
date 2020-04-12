# Octomap建图

## 1. Octomap 介绍
点云地图具有占据空间大，更新不够方便的特点，这决定了其难以直接用于导航使用，而Octomap就是一种基于稠密点云的新的地图表达方式，相比点云而言，具有占据空间小，可以方便的更新的优点，可以用于机器人的导航。

![GLOZfP.png](https://s1.ax1x.com/2020/04/12/GLOZfP.png)

Octomap通过讲正方体空间划分为八个小正方体，从而构造成一个八叉树的结构，大的正方体成为父节点，小的正方体成为子节点，八叉树可以不断地向下拓展，直到达到最小分辨率，称为叶节点

![GLOB79.png](https://s1.ax1x.com/2020/04/12/GLOB79.png)

Octomap使用概率描述节点的占据状态，概率大于0.5表示占据，小于0.5表示未占据，根据其更新算法可以方便的根据传感器数据对节点状态进行更新。同时，当所有子节点状态相同时，可将其中的所有子节点删除，只用父节点表示该空间状态，这种方式大量的节省了存储空间。

## 2. 仿真流程

### 数据集

配一个3dlidar的数据集,并附上运行方法和运行截图

### obstacle.world 建图

- 运行启动脚本,并耐心等待Gazebo及rviz启动
    	roslaunch prometheus_gazebo sitl_octomap.launch
- 正常运行截图如下,在rviz中勾选Ground_Truth及Octomap建图显示选项,绿色为真值,白色为建图结果(全局点云),彩色图案为octomap格式地图

	[![GL4LdK.md.png](https://s1.ax1x.com/2020/04/12/GL4LdK.md.png)](https://imgchr.com/i/GL4LdK)
	[![GLTyfU.md.png](https://s1.ax1x.com/2020/04/12/GLTyfU.md.png)](https://imgchr.com/i/GLTyfU)
- 可使用terminal或键盘控制无人机运动,查看实时建图结果
		rosrun prometheus_control terminal_control





