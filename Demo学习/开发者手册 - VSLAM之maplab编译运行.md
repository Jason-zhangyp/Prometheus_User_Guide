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