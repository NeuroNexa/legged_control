# 指定CMake的最低版本要求
cmake_minimum_required(VERSION 3.10)
# 定义项目名称
project(legged_gazebo)

# 开启导出编译命令的功能
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 查找catkin包及其组件
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        legged_common
        gazebo_dev         # Gazebo开发库
        gazebo_ros_control # Gazebo的ros_control插件
        )

# 配置catkin包的导出信息
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES legged_hw_sim # 导出的库文件
        CATKIN_DEPENDS
        roscpp
        legged_common
        gazebo_ros_control
        # DEPENDS
)

###########
## Build ##
###########

# 添加头文件搜索路径
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS} # Gazebo的头文件路径
)

# 声明一个C++库，这个库就是硬件仿真接口的实现
add_library(legged_hw_sim
        src/LeggedHWSim.cpp
        )

# 添加依赖
add_dependencies(legged_hw_sim
        ${catkin_EXPORTED_TARGETS}
        )

# 为库链接依赖的库
target_link_libraries(legged_hw_sim
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES} # Gazebo的库文件
        )

#########################
###   CLANG TOOLING   ###
#########################
# 查找并配置clang工具
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target legged_hw_sim")
    add_clang_tooling(
            TARGETS legged_hw_sim
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# 安装目标库文件
install(TARGETS legged_hw_sim
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# 安装C++头文件
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

# 安装插件描述文件
install(FILES legged_hw_sim_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

# 安装config目录
install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
