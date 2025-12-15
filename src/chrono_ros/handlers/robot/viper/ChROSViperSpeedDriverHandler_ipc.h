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
// IPC data structure for ViperSpeedDriver handler
// This file defines the POD type shared between the main process and the ROS
// subprocess. Keep it free of ROS/Chrono symbols and dynamic allocations.
//
// =============================================================================

#ifndef CH_ROS_VIPER_SPEED_DRIVER_HANDLER_IPC_H
#define CH_ROS_VIPER_SPEED_DRIVER_HANDLER_IPC_H

#include <cstdint>

namespace chrono {
namespace ros {
namespace ipc {

/// Simple IPC struct carrying a single speed command.
struct ViperSpeedCommand {
    double speed;
};

}  // namespace ipc
}  // namespace ros
}  // namespace chrono

#endif

