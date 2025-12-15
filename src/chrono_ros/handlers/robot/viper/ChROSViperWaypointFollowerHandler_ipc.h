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
// IPC data structure for ViperWaypointFollower handler
// Plain-old-data only; safe for both main process and ROS subprocess.
//
// =============================================================================

#ifndef CH_ROS_VIPER_WAYPOINT_FOLLOWER_HANDLER_IPC_H
#define CH_ROS_VIPER_WAYPOINT_FOLLOWER_HANDLER_IPC_H

#include <cstdint>

namespace chrono {
namespace ros {
namespace ipc {

/// IPC struct carrying a single waypoint target.
struct ViperWaypointTarget {
    double x;
    double y;
    double z;
};

}  // namespace ipc
}  // namespace ros
}  // namespace chrono

#endif

