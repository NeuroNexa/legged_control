# 指定CMake的最低版本要求
cmake_minimum_required(VERSION 3.10)
# 定义项目名称
project(legged_common)

# 开启导出编译命令的功能，用于生成compile_commands.json，方便IDE和静态分析工具使用
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 查找catkin包，这是ROS构建系统的核心
# COMPONENTS指定了该项目依赖的其他catkin包
find_package(catkin REQUIRED
        COMPONENTS
        roscpp  # ROS C++客户端库
        hardware_interface # ROS Control的硬件抽象接口
        )

# 配置catkin包的导出信息
catkin_package(
        INCLUDE_DIRS include # 导出的头文件目录
        # LIBRARIES # 本项目不产生库文件，所以这里为空
        CATKIN_DEPENDS # catkin依赖
        roscpp
        hardware_interface
        # DEPENDS # 其他系统依赖，这里为空
)

# 添加头文件搜索路径
include_directories(
        include # 项目自身的include目录
        ${catkin_INCLUDE_DIRS} # catkin提供的依赖包的include目录
)

# 添加一个INTERFACE库，这是一种特殊的库，它本身不编译源文件，只用于传递编译属性给其他目标
add_library(${PROJECT_NAME} INTERFACE)

# 为目标设置头文件包含目录，INTERFACE意味着任何链接到此目标的目标都将自动获得这些包含目录
target_include_directories(${PROJECT_NAME} INTERFACE include)

#########################
###   CLANG TOOLING   ###
#########################
# 查找cmake_clang_tools包，用于代码静态分析和格式化
find_package(cmake_clang_tools QUIET) # QUIET参数表示找不到也不会报错
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    # 添加clang-tidy和clang-format的配置
    add_clang_tooling(
            TARGETS ${PROJECT_NAME} # 应用于哪个目标
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include # 源码和头文件目录
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include # 头文件目录
            CF_WERROR # 将clang-format的警告视为错误
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# 安装目标文件
install(
        TARGETS ${PROJECT_NAME} # 要安装的目标
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} # 静态库安装路径
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} # 动态库安装路径
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} # 可执行文件安装路径
)

# 安装C++头文件
install(
        DIRECTORY include/${PROJECT_NAME}/ # 要安装的目录
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} # 安装目标路径
        FILES_MATCHING PATTERN "*.h" # 只安装.h文件
)

# 安装可执行脚本
install(PROGRAMS scripts/generate_urdf.sh # 要安装的脚本
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts # 安装目标路径
        )
