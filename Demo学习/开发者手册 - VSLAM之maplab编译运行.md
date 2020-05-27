# VSLAM之maplab篇

## maplab介绍

[maplab](https://github.com/ethz-asl/maplab)是ETH的ASL团队在2018年开源的一个视觉+惯性导航的建图框架，可以实现多场景建图的创建、处理和操作。它包含了两部分：在线的前端ROVIOLI用于创建地图，可以实现建图和定位功能；离线的地图处理包maplab_console，可以为生成的地图进行二次处理（生成地图优化、组合和建立稠密地图等）。其工作流程如下图所示

![tAMUG4.png](https://s1.ax1x.com/2020/05/27/tAMUG4.png)

- (a)  在VIO模式下使用ROVIOLI进行建图。

- (b)  使用maplab console对地图进行优化，实现不同地图的融合。

- (c)  在定位模式下运行ROVIOLI，定位的功能提高了视觉+惯性导航的位姿估计精度。

## maplab的编译

编译maplab遇到问题：

- 下载依赖遇到问题：

https://github.com/ethz-asl/maplab/wiki/FAQ#q-why-do-i-get-missing-dependencies-when-building-the-maplab-workspace

从cmkaelist中获取源码下载链接，然后解压缩到相应路径

- 编译opencv3_catkin遇到问题

~/maplab_ws/build/opencv3_catkin/opencv3_src/cmake/OpenCVCompilerOptions.cmake和
/usr/local/libcmake/Ceres/CeresConfig.cmake报以下错误：

    A duplicate ELSE command was found inside an IF block.

注释掉对应行即可

- 遇到brisk-detector源码问题：解决方法参照https://github.com/ethz-asl/ethzasl_brisk/pull/109，在brisk-feature-detector.cc中加入包含#include <functional>