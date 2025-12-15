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
// ROS publishing implementation for ViperWaypointPath handler (subprocess).
// Receives serialized path data from the main process, deserializes it, and
// publishes nav_msgs/Path.
//
// =============================================================================

#include "chrono_ros/ChROSHandlerRegistry.h"
#include "chrono_ros/handlers/robot/viper/ChROSViperWaypointPathHandler_ipc.h"
#include "chrono_ros/ipc/ChROSIPCChannel.h"
#include "chrono_ros/ipc/ChROSIPCMessage.h"

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <unordered_map>
#include <cstring>

namespace chrono {
namespace ros {

/// Publish path data to ROS.
void PublishViperWaypointPathToROS(const uint8_t* data,
                                   size_t data_size,
                                   rclcpp::Node::SharedPtr node,
                                   ipc::IPCChannel* channel) {
    (void)channel;

    if (data_size < sizeof(ipc::ViperWaypointPathHeader)) {
        RCLCPP_ERROR(node->get_logger(), "WaypointPathHandler: data too small (%zu)", data_size);
        return;
    }

    ipc::ViperWaypointPathHeader header{};
    std::memcpy(&header, data, sizeof(header));

    const size_t expected_points_bytes =
        static_cast<size_t>(header.point_count) * sizeof(ipc::ViperWaypointPathPoint);
    const size_t expected_total = sizeof(header) + expected_points_bytes;
    if (data_size < expected_total) {
        RCLCPP_ERROR(node->get_logger(), "WaypointPathHandler: size mismatch (%zu < %zu)", data_size, expected_total);
        return;
    }

    const auto* points =
        reinterpret_cast<const ipc::ViperWaypointPathPoint*>(data + sizeof(ipc::ViperWaypointPathHeader));

    static std::unordered_map<std::string, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> publishers;
    std::string topic(header.topic_name);

    if (publishers.find(topic) == publishers.end()) {
        publishers[topic] = node->create_publisher<nav_msgs::msg::Path>(topic, 1);
        RCLCPP_INFO(node->get_logger(), "Created waypoint path publisher on topic %s", topic.c_str());
    }

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = node->get_clock()->now();
    path_msg.header.frame_id = std::string(header.frame_id);
    path_msg.poses.reserve(header.point_count);

    for (uint32_t i = 0; i < header.point_count; ++i) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = points[i].x;
        pose.pose.position.y = points[i].y;
        pose.pose.position.z = points[i].z;
        pose.pose.orientation.w = 1.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        path_msg.poses.push_back(pose);
    }

    publishers[topic]->publish(path_msg);
}

CHRONO_ROS_REGISTER_HANDLER(VIPER_WAYPOINT_PATH, PublishViperWaypointPathToROS)

}  // namespace ros
}  // namespace chrono

