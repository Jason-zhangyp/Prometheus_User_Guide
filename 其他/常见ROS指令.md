# 常见ROS指令

## rosbag的使用

### rosbag简介

rosbag 既可以指命令行中数据包相关命令，也可以指 c++/python 的 rosbag 库。这里的 rosbag 是指前者。rosbag 主要用于记录、回放、分析 rostopic 中的数据。它可以将指定 rostopic 中的数据记录到 .bag 后缀的数据包中，便于对其中的数据进行离线分析和处理。对于 subscribe 某个 topic 的节点来说，它无法区分这个 topic 中的数据到底是实时获取的数据还是从 rosbag 中回放的数据。这就有助于我们基于离线数据快速重现曾经的实际场景，进行可重复、低成本的分析和调试。

### rosbag record

启动需要被记录的相关节点,如

```c
roslaunch prometheus_gazebo sitl.launch
```

打开需要保存的路径,可以选择录制所有话题,-a 选项表示将当前发布的所有 topic 数据都录制保存到一个 rosbag 文件中,，录制的数据包名字为日期加时间。也可以选择只录制指定的话题

``` c
cd ~/rosbag_files
// 录制所有话题
rosbag record -a 
// 录制指定的话题
rosbag record /topic_name1 /topic_name2 /topic_name3
```

如果要指定生成数据包的名字，则用-O /-o 参数，如下：

``` c
rosbag record -O filename.bag /topic_name1
```

如果在 launch 文件中使用 rosbag record 命令，如下：

``` c
<node pkg="rosbag" type="record" name="bag_record" args="/topic1 /topic2"/> 
```

### rosbag info

rosbag info指令可以显示数据包中的信息:

``` c  
 rosbag info filename.bag
```

### rosbag play

回放数据包中的 topic

``` c
rosbag play <bagfile>
```

如果想改变消息的发布速率，可以用下面的命令，-r 后面的数字对应播放速率

``` c
rosbag play -r 2 <bagfile>
```

如果希望 rosbag 循环播放，可以用命令

``` c
rosbag play -l  <bagfile>  # -l== --loop
```

如果只播放感兴趣的 topic ，则用命令

``` c
rosbag play <bagfile> --topic /topic1
```

在上述播放命令执行期间，空格键可以暂停播放。

