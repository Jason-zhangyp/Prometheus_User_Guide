# Octomap建图

## 1. Octomap 介绍
点云地图具有占据空间大，更新不够方便的特点，这决定了其难以直接用于导航使用，而Octomap就是一种基于稠密点云的新的地图表达方式，相比点云而言，具有占据空间小，可以方便的更新的优点，可以用于机器人的导航。

![GLOZfP.png](https://s1.ax1x.com/2020/04/12/GLOZfP.png)

Octomap通过将正方体空间划分为八个小正方体，从而构造成一个八叉树的结构，大的正方体成为父节点，小的正方体成为子节点，八叉树可以不断地向下拓展，直到达到最小分辨率，称为叶节点

![GLOB79.png](https://s1.ax1x.com/2020/04/12/GLOB79.png)

Octomap使用概率描述节点的占据状态，概率大于0.5表示占据，小于0.5表示未占据，使用概率表示占据信息的的方法可以方便的根据传感器数据对节点状态进行更新。同时，当所有子节点状态相同时，可将其中的所有子节点删除，只用父节点表示该空间状态，这种方式大量的节省了存储空间。

## 2. 仿真流程

### 数据集建图

可通过该[链接](http://ais.informatik.uni-freiburg.de/projects/datasets/octomap/)下载Octomap数据集，数据集中包含室外的激光点云数据和室内的RGBD数据，下载完解压后可以使用graph2tree工具对数据进行建图：

```
graph2tree -i input.graph -o map.bt -res 0.1
```

建图完成后会形成.bt和.ot两种格式的地图文件，其中.ot包含所有占据信息，而.bt文件压缩程度更高。使用octoviz工具查看地图

```
sudo apt install octovis
octovis map.ot
octovis map.bt
```

以下结果分别是激光点云和RGBD的结果：

![GOEy4K.png](https://s1.ax1x.com/2020/04/12/GOEy4K.png)

![GOErAx.png](https://s1.ax1x.com/2020/04/12/GOErAx.png)

### obstacle.world 建图

- 运行启动脚本,并耐心等待Gazebo及rviz启动
    	roslaunch prometheus_gazebo sitl_octomap.launch
    	
- 在launch中可选择使用Lidar还是RGBD作为输入进行建图。使用octomap_server进行ros下的建图，需要分别设置分辨率、世界坐标系、传感器范围和点云的话题名称。
    	
- 正常运行截图如下,在rviz中勾选Ground_Truth及Octomap建图显示选项,绿色为真值,白色为建图结果(全局点云),彩色图案为octomap格式地图

	[![GL4LdK.md.png](https://s1.ax1x.com/2020/04/12/GL4LdK.md.png)](https://imgchr.com/i/GL4LdK)
	[![GLTyfU.md.png](https://s1.ax1x.com/2020/04/12/GLTyfU.md.png)](https://imgchr.com/i/GLTyfU)
	
- 可使用terminal或键盘控制无人机运动,查看实时建图结果，运行效果如下：
		rosrun prometheus_control terminal_control

![GOZBY6.png](https://s1.ax1x.com/2020/04/12/GOZBY6.png)



