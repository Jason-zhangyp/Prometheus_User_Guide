# Octomap建图
## 1. Octomap 介绍
简单介绍下octomap即可,200字以内
## 2. 仿真流程
### 数据集建图

配一个3dlidar的数据集,并附上运行方法和运行截图

### obstacle.world 建图

- 运行启动脚本,并耐心等待Gazebo及rviz启动
    	roslaunch prometheus_gazebo sitl_octomap.launch
- 正常运行截图如下,在rviz中勾选Ground_Truth及Octomap建图显示选项,绿色为真值,白色为建图结果(全局点云),彩色图案为octomap格式地图

	[![GL4LdK.md.png](https://s1.ax1x.com/2020/04/12/GL4LdK.md.png)](https://imgchr.com/i/GL4LdK)
	[![GLTyfU.md.png](https://s1.ax1x.com/2020/04/12/GLTyfU.md.png)](https://imgchr.com/i/GLTyfU)
- 可使用terminal或键盘控制无人机运动,查看实时建图结果
		rosrun prometheus_control terminal_control





