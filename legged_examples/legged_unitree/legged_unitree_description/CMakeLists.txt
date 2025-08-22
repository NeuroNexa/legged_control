# 指定CMake的最低版本要求
cmake_minimum_required(VERSION 3.10)
# 定义项目名称
project(legged_unitree_description)

# 查找catkin包
find_package(catkin REQUIRED
        COMPONENTS
        )

###################################
## catkin specific configuration ##
###################################
# 配置catkin包的导出信息
# 这个包只包含模型文件(URDF, mesh)和启动文件，没有库或头文件，所以大部分是空的
catkin_package(
        # CATKIN_DEPENDS
)

#############
## Install ##
#############

# 安装资源文件
# 将 meshes, urdf, 和 launch 目录安装到包的 `share` 目录下
# 这样ROS的`find`命令才能在运行时找到这些文件
install(
        DIRECTORY meshes urdf launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
