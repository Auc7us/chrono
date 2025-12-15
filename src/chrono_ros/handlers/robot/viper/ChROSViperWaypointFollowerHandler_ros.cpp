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
// ROS subscriber implementation for ViperWaypointFollower handler (subprocess).
// Subscribes to geometry_msgs/Vector3 and forwards waypoint targets to the main
// process via IPC.
//
// =============================================================================

#include "chrono_ros/ChROSHandlerRegistry.h"
#include "chrono_ros/ipc/ChROSIPCChannel.h"
#include "chrono_ros/ipc/ChROSIPCMessage.h"
#include "chrono_ros/handlers/robot/viper/ChROSViperWaypointFollowerHandler_ipc.h"

#include "geometry_msgs/msg/vector3.hpp"
#include <cstring>

namespace chrono {
namespace ros {

static rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr g_waypoint_subscriber;
static ipc::IPCChannel* g_ipc_channel = nullptr;

/// Callback invoked when waypoint target arrives from ROS.
void OnViperWaypointTarget(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    if (!g_ipc_channel) {
        RCLCPP_WARN(rclcpp::get_logger("ChROSViperWaypointFollower"), "IPC channel not ready, skipping waypoint");
        return;
    }

    ipc::ViperWaypointTarget data;
    data.x = msg->x;
    data.y = msg->y;
    data.z = msg->z;

    static ipc::Message ipc_msg;
    ipc_msg.header = ipc::MessageHeader(ipc::MessageType::VIPER_WAYPOINT_FOLLOWER, 0, sizeof(data), 0);
    std::memcpy(ipc_msg.payload.get(), &data, sizeof(data));

    g_ipc_channel->SendMessage(ipc_msg);
}

/// Setup function called by dispatcher to create subscriber.
void SetupViperWaypointFollowerSubscriber(const uint8_t* data,
                                          size_t data_size,
                                          rclcpp::Node::SharedPtr node,
                                          ipc::IPCChannel* channel) {
    std::string topic_name;
    if (data_size > 0) {
        topic_name = std::string(reinterpret_cast<const char*>(data), data_size);
    } else {
        topic_name = "~/input/viper_waypoint";
    }

    g_ipc_channel = channel;

    if (!g_waypoint_subscriber) {
        g_waypoint_subscriber =
            node->create_subscription<geometry_msgs::msg::Vector3>(topic_name, 10, OnViperWaypointTarget);
    }
}

CHRONO_ROS_REGISTER_HANDLER(VIPER_WAYPOINT_FOLLOWER, SetupViperWaypointFollowerSubscriber)

}  // namespace ros
}  // namespace chrono

