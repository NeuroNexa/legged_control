# 指定CMake的最低版本要求
cmake_minimum_required(VERSION 3.1.3)
# 定义项目名称
project(legged_control)
# 查找catkin包，这是ROS构建系统的核心
find_package(catkin REQUIRED)
# 将此包声明为一个元包（metapackage）
# 元包本身不包含代码或文件，它只用来组合一组相关的包。
catkin_metapackage()
