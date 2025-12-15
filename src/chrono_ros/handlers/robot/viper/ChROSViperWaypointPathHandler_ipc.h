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
// IPC data structures for ViperWaypointPath handler (Chrono → ROS publisher)
// Contains only POD types to keep it safe for serialization and shared use.
//
// =============================================================================

#ifndef CH_ROS_VIPER_WAYPOINT_PATH_HANDLER_IPC_H
#define CH_ROS_VIPER_WAYPOINT_PATH_HANDLER_IPC_H

#include <cstdint>

namespace chrono {
namespace ros {
namespace ipc {

/// Header describing the waypoint path payload.
struct ViperWaypointPathHeader {
    char topic_name[128];
    char frame_id[64];
    uint32_t point_count;
};

/// One waypoint in the path.
struct ViperWaypointPathPoint {
    double x;
    double y;
    double z;
};

}  // namespace ipc
}  // namespace ros
}  // namespace chrono

#endif

