#!/usr/bin/env sh
# 这是一个shell脚本，用于从xacro文件生成URDF文件。

# 创建一个临时目录来存放生成的URDF文件
# -p 选项表示如果目录已存在，则不报错
mkdir -p /tmp/legged_control/

# 使用xacro工具将xacro文件转换为URDF文件
# $1 是脚本的第一个参数，应该是xacro文件的路径
# $2 是脚本的第二个参数，应该是机器人类型（例如 a1, aliengo）
# robot_type:=$2 将robot_type参数传递给xacro
# > /tmp/legged_control/$2.urdf 将输出重定向到临时目录中的一个以机器人类型命名的URDF文件
rosrun xacro xacro $1 robot_type:=$2 > /tmp/legged_control/$2.urdf
