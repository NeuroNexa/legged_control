# 指定CMake的最低版本要求
cmake_minimum_required(VERSION 3.10)
# 定义项目名称
project(legged_hw)

# 开启导出编译命令的功能
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 查找catkin包及其组件
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        legged_common
        controller_manager # ROS控制的核心包
        urdf               # 用于解析URDF文件
        )

# 配置catkin包的导出信息
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        legged_common
        controller_manager
        urdf
)

###########
## Build ##
###########

# 添加头文件搜索路径
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

# 声明一个C++库，包含硬件接口基类和控制循环的实现
add_library(${PROJECT_NAME}
        src/LeggedHW.cpp
        src/LeggedHWLoop.cpp
        )

# 为库链接依赖的库
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

#########################
###   CLANG TOOLING   ###
#########################
# 查找并配置clang工具
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# 安装目标库文件
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# 安装C++头文件
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )
