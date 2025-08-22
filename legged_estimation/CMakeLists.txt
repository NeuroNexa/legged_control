# 指定CMake的最低版本要求
cmake_minimum_required(VERSION 3.10)
# 定义项目名称
project(legged_estimation)

# 开启导出编译命令的功能
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 查找catkin包及其组件
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        ocs2_legged_robot
        legged_common
        realtime_tools
        tf2_ros
        )

# 使用PkgConfig查找pinocchio库
find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

# 配置catkin包的导出信息
catkin_package(
        INCLUDE_DIRS include # 导出的头文件目录
        LIBRARIES ${PROJECT_NAME} # 导出的库文件
        CATKIN_DEPENDS # Catkin依赖
        roscpp
        ocs2_legged_robot
        legged_common
        realtime_tools
        tf2_ros
)

###########
## Build ##
###########

# 添加头文件搜索路径
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${pinocchio_INCLUDE_DIRS}
)

# 添加链接库的搜索路径
link_directories(
        ${pinocchio_LIBRARY_DIRS}
)

# 声明一个C++库，包含所有状态估计器的实现
add_library(${PROJECT_NAME}
        src/StateEstimateBase.cpp
        src/FromTopicEstimate.cpp
        src/LinearKalmanFilter.cpp
        )

# 为库链接依赖的库
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${pinocchio_LIBRARIES}
        )

# 为目标设置编译选项，使用OCS2的C++标志
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})


# 安装目标文件
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
