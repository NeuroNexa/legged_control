# 指定CMake的最低版本要求
cmake_minimum_required(VERSION 3.10)
# 定义项目名称
project(legged_unitree_hw)

# 开启导出编译命令的功能
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 查找catkin包及其组件
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        legged_hw
)

# 配置catkin包的导出信息
catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS
        roscpp
        legged_hw
)

###########
## Build ##
###########

# 添加头文件和链接库的搜索路径
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)
link_directories(lib)

# 设置源文件列表
set(SOURCE src/legged_unitree_hw.cpp src/UnitreeHW.cpp)


# --- 为Unitree SDK 3.3.1版本构建可执行文件 ---
# 使用相同的源文件，但定义了特定的宏并链接了对应的SDK库
add_executable(${PROJECT_NAME}_3_3_1
        ${SOURCE}
        )

# 添加一个预处理器定义，这样可以在代码中使用 #ifdef UNITREE_SDK_3_3_1 来区分不同版本的代码
target_compile_definitions(${PROJECT_NAME}_3_3_1 PRIVATE UNITREE_SDK_3_3_1)

# 链接catkin库、Unitree SDK 3.3.1的共享库和lcm库
target_link_libraries(${PROJECT_NAME}_3_3_1
        ${catkin_LIBRARIES}
        libunitree_legged_sdk_amd64_3_3_1.so lcm
        )


# --- 为Unitree SDK 3.8.0版本构建可执行文件 ---
# 同样使用相同的源文件
add_executable(${PROJECT_NAME}_3_8_0
        ${SOURCE}
)

# 添加不同的预处理器定义
target_compile_definitions(${PROJECT_NAME}_3_8_0 PRIVATE UNITREE_SDK_3_8_0)

# 链接catkin库和Unitree SDK 3.8.0的静态库
target_link_libraries(${PROJECT_NAME}_3_8_0
        ${catkin_LIBRARIES}
        libunitree_legged_sdk_amd64_3_8_0.a
)


#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    # 对两个目标都应用clang-tidy和clang-format
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}_3_3_1 TARGETS ${PROJECT_NAME}_3_8_0
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include/legged_unitree_hw
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/legged_unitree_hw
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)


#############
## Install ##
#############

# 安装两个可执行文件
install(TARGETS ${PROJECT_NAME}_3_3_1 ${PROJECT_NAME}_3_8_0
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# 安装头文件 (注意：这里的目录名可能不正确，因为include下没有这两个子目录)
install(DIRECTORY include/${PROJECT_NAME}_3_3_1/ include/${PROJECT_NAME}_3_8_0/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

# 安装其他资源文件
install(DIRECTORY config launch lib
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
