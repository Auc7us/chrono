// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Patrick Chen
// =============================================================================
//
// ROS subscriber implementation for ViperSpeedDriver handler (subprocess side).
// Creates a subscription to std_msgs/Float32 and forwards commands to the main
// process via IPC.
//
// =============================================================================

#include "chrono_ros/ChROSHandlerRegistry.h"
#include "chrono_ros/ipc/ChROSIPCMessage.h"
#include "chrono_ros/ipc/ChROSIPCChannel.h"
#include "chrono_ros/handlers/robot/viper/ChROSViperSpeedDriverHandler_ipc.h"

#include "std_msgs/msg/float32.hpp"
#include <cstring>

namespace chrono {
namespace ros {

static rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr g_speed_subscriber;
static ipc::IPCChannel* g_ipc_channel = nullptr;

/// Callback when a speed command arrives from ROS.
void OnViperSpeedCommand(const std_msgs::msg::Float32::SharedPtr msg) {
    if (!g_ipc_channel) {
        RCLCPP_WARN(rclcpp::get_logger("ChROSViperSpeedDriver"), "IPC channel not ready, skipping speed command");
        return;
    }

    ipc::ViperSpeedCommand data;
    data.speed = msg->data;

    static ipc::Message ipc_msg;
    ipc_msg.header = ipc::MessageHeader(ipc::MessageType::VIPER_SPEED_DRIVER, 0, sizeof(data), 0);
    std::memcpy(ipc_msg.payload.get(), &data, sizeof(data));

    g_ipc_channel->SendMessage(ipc_msg);
}

/// Setup function invoked by dispatcher when the main process sends topic name.
void SetupViperSpeedDriverSubscriber(const uint8_t* data,
                                     size_t data_size,
                                     rclcpp::Node::SharedPtr node,
                                     ipc::IPCChannel* channel) {
    std::string topic_name;
    if (data_size > 0) {
        topic_name = std::string(reinterpret_cast<const char*>(data), data_size);
    } else {
        topic_name = "~/input/viper_speed";
    }

    g_ipc_channel = channel;

    if (!g_speed_subscriber) {
        g_speed_subscriber =
            node->create_subscription<std_msgs::msg::Float32>(topic_name, 10, OnViperSpeedCommand);
    }
}

CHRONO_ROS_REGISTER_HANDLER(VIPER_SPEED_DRIVER, SetupViperSpeedDriverSubscriber)

}  // namespace ros
}  // namespace chrono

