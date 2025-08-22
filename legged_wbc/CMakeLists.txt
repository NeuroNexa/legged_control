# 指定CMake的最低版本要求
cmake_minimum_required(VERSION 3.10)
# 定义项目名称
project(legged_wbc)

# 开启导出编译命令的功能
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 查找catkin包及其组件
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        ocs2_legged_robot
        qpoases_catkin     # qpOASES求解器的封装包
        )

# 查找Eigen3和Pinocchio库
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

# 配置catkin包的导出信息
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        ocs2_legged_robot
        DEPENDS
        pinocchio
)

###########
## Build ##
###########

# 添加头文件和链接库的搜索路径
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${pinocchio_INCLUDE_DIRS}
)
link_directories(
        ${pinocchio_LIBRARY_DIRS}
)

# 声明一个C++库，包含WBC的各种实现
add_library(${PROJECT_NAME}
        src/HoQp.cpp
        src/WbcBase.cpp
        src/HierarchicalWbc.cpp
        src/WeightedWbc.cpp
        )

# 为库链接依赖的库
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${pinocchio_LIBRARIES}
        )

# 设置编译选项
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
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

#############
## Testing ##
#############
# 如果启用了测试
if (CATKIN_ENABLE_TESTING)
    # 添加一个GTest测试目标
    catkin_add_gtest(${PROJECT_NAME}_test
            test/HoQp_test.cpp
            )

    # 为测试目标链接被测试的库和其他依赖
    target_link_libraries(${PROJECT_NAME}_test
            ${PROJECT_NAME}
            ${pinocchio_LIBRARIES}
            gtest_main
            )

    # 设置测试目标的编译选项
    target_compile_options(${PROJECT_NAME}_test PUBLIC ${FLAGS})
endif (CATKIN_ENABLE_TESTING)
