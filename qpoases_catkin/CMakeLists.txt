# 指定CMake的最低版本要求
cmake_minimum_required(VERSION 3.14)
# 定义项目名称
project(qpoases_catkin)

# 查找catkin包
find_package(catkin REQUIRED)

# 包含FetchContent模块，用于在配置时下载和构建外部项目
include(FetchContent)

# --- 定义目录 ---
# 定义qpOASES的开发、包含、库、下载和构建目录
set(QPOASES_DEVEL_PREFIX ${CATKIN_DEVEL_PREFIX} CACHE STRING "QPOASES install path")
set(QPOASES_INCLUDE_DIR ${QPOASES_DEVEL_PREFIX}/include)
set(QPOASES_LIB_DIR ${QPOASES_DEVEL_PREFIX}/lib)
set(QPOASES_DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/download)
set(QPOASES_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/build)

# 确保这些目录存在
file(MAKE_DIRECTORY ${QPOASES_INCLUDE_DIR})
file(MAKE_DIRECTORY ${QPOASES_LIB_DIR})
file(MAKE_DIRECTORY ${QPOASES_DOWNLOAD_DIR})
file(MAKE_DIRECTORY ${QPOASES_BUILD_DIR})

# --- qpOASES 构建设置 ---
set(BUILD_SHARED_LIBS ON CACHE STRING "Build shared libraries" FORCE)
set(QPOASES_BUILD_EXAMPLES OFF CACHE BOOL "Examples disable")

# --- 下载并构建源码 ---
# 声明一个名为 qpoasesDownload 的 FetchContent 内容
FetchContent_Declare(qpoasesDownload
        # Git仓库地址
        GIT_REPOSITORY https://github.com/coin-or/qpOASES
        # 指定一个特定的commit hash，以确保版本一致性
        GIT_TAG 268b2f2659604df27c82aa6e32aeddb8c1d5cc7f
        # 不执行更新命令
        UPDATE_COMMAND ""
        # 源码下载路径
        SOURCE_DIR ${QPOASES_DOWNLOAD_DIR}
        # 构建路径
        BINARY_DIR ${QPOASES_BUILD_DIR}
        # 构建命令
        BUILD_COMMAND $(MAKE)
        # 安装命令
        INSTALL_COMMAND "$(MAKE) install"
        )
# 使下载的内容可用，这将触发下载、构建和安装过程
FetchContent_MakeAvailable(qpoasesDownload)

# --- 将构建产物复制到catkin期望的位置 ---
# 因为qpOASES使用Makefile而不是CMake，FetchContent可能无法正确地将头文件和库文件
# 放到catkin的工作空间中，所以需要手动复制。

# 复制主头文件
file(COPY ${QPOASES_DOWNLOAD_DIR}/include/qpOASES.hpp DESTINATION ${QPOASES_INCLUDE_DIR})

# 递归地查找并复制所有子目录中的头文件
file(GLOB_RECURSE HEADERS "${QPOASES_DOWNLOAD_DIR}/include/qpOASES/*")
foreach (HEADER_FILE ${HEADERS})
    message(STATUS "FOUND HEADER: " ${HEADER_FILE})
    file(COPY ${HEADER_FILE} DESTINATION ${QPOASES_INCLUDE_DIR}/qpOASES)
endforeach ()

file(GLOB_RECURSE HEADERS "${QPOASES_DOWNLOAD_DIR}/include/qpOASES/extras/*")
foreach (HEADER_FILE ${HEADERS})
    message(STATUS "FOUND HEADER: " ${HEADER_FILE})
    file(COPY ${HEADER_FILE} DESTINATION ${QPOASES_INCLUDE_DIR}/qpOASES/extras)
endforeach ()

# 设置qpOASES目标的库文件输出目录
set_target_properties(qpOASES PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${QPOASES_LIB_DIR})

# --- 配置catkin包 ---
# 告诉catkin这个包导出了哪些头文件目录和库
catkin_package(
        INCLUDE_DIRS ${QPOASES_INCLUDE_DIR}
        LIBRARIES qpOASES
)
