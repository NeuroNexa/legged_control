# 指定CMake的最低版本要求
cmake_minimum_required(VERSION 3.10)
# 定义项目名称
project(legged_controllers)

# 开启导出编译命令的功能
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 查找catkin包及其组件
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        legged_common
        legged_interface
        legged_wbc
        legged_estimation
        controller_interface
        ocs2_legged_robot_ros
        ocs2_self_collision_visualization
        angles
        )

# 查找PkgConfig，用于查找系统安装的库
find_package(PkgConfig REQUIRED)
# 使用PkgConfig查找pinocchio库
pkg_check_modules(pinocchio REQUIRED pinocchio)

# 配置catkin包的导出信息
catkin_package(
        INCLUDE_DIRS include # 导出的头文件目录
        LIBRARIES ${PROJECT_NAME} # 导出的库文件
        CATKIN_DEPENDS # Catkin依赖
        roscpp
        legged_common
        legged_interface
        legged_wbc
        legged_estimation
        controller_interface
        ocs2_legged_robot_ros
        ocs2_self_collision_visualization
        angles
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

# 声明一个C++库
add_library(${PROJECT_NAME}
        src/LeggedController.cpp
        )

# 为库链接依赖的库
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${pinocchio_LIBRARIES}
        )

# 为目标设置编译选项，使用OCS2的C++标志
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})


# 声明一个可执行文件：目标轨迹发布节点
add_executable(legged_target_trajectories_publisher
        src/TargetTrajectoriesPublisher.cpp
        )
# 为可执行文件添加依赖
add_dependencies(legged_target_trajectories_publisher
        ${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        )
# 为可执行文件链接库
target_link_libraries(legged_target_trajectories_publisher
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )
# 为可执行文件设置私有的编译选项
target_compile_options(legged_target_trajectories_publisher PRIVATE ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
# 查找并配置clang工具
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME} legged_target_trajectories_publisher
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# 安装目标文件（库和可执行文件）
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

# 安装插件配置文件
install(FILES legged_controllers_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
