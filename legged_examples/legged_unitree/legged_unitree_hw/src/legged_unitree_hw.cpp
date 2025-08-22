/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by qiayuan on 12/27/20.
//

#include "legged_unitree_hw/UnitreeHW.h"
#include <legged_hw/LeggedHWLoop.h>

/**
 * @brief 主函数
 *
 * 这是Unitree硬件接口节点（`legged_unitree_hw`）的入口点。
 * 它负责初始化ROS，创建并运行一个`ros_control`的硬件接口循环。
 */
int main(int argc, char** argv) {
  // 初始化ROS节点
  ros::init(argc, argv, "legged_unitree_hw");
  ros::NodeHandle nh;
  ros::NodeHandle robotHwNh("~"); // 私有节点句柄，用于获取该节点的特定参数

  // --- 运行硬件接口节点 ---

  // 我们在一个单独的线程中运行ROS事件循环（spinner），
  // 因为一些外部调用（如加载控制器的服务回调）可能会阻塞主（控制）循环。
  ros::AsyncSpinner spinner(3); // 使用3个线程
  spinner.start();

  try {
    // 1. 创建特定于机器人的硬件接口
    std::shared_ptr<legged::UnitreeHW> unitreeHw = std::make_shared<legged::UnitreeHW>();

    // 2. 初始化硬件接口:
    //    a. 从rosparam检索配置
    //    b. 初始化硬件并将其与ros_control接口连接
    unitreeHw->init(nh, robotHwNh);

    // 3. 启动控制循环
    //    LeggedHWLoop负责以固定频率调用硬件接口的read()和write()方法，
    //    并管理控制器管理器（controller_manager）。
    legged::LeggedHWLoop controlLoop(nh, unitreeHw);

    // 4. 等待直到收到关闭信号 (例如, Ctrl+C)
    ros::waitForShutdown();
  } catch (const ros::Exception& e) {
    ROS_FATAL_STREAM("Error in the hardware interface:\n"
                     << "\t" << e.what());
    return 1;
  }

  return 0;
}
