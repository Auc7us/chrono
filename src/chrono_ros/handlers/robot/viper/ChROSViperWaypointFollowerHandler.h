// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Keshav Sharan
// =============================================================================
//
// Handler responsible for receiving and updating the DC motor control commands
// for the Viper rover
//
// =============================================================================

#ifndef CH_VIPER_WAYPOINT_FOLLOWER_HANDLER_H
#define CH_VIPER_WAYPOINT_FOLLOWER_HANDLER_H

#include "chrono_ros/ChROSHandler.h"
#include "chrono_models/robot/viper/Viper.h"
#include "chrono_ros/handlers/robot/viper/ChROSViperWaypointFollowerHandler_ipc.h"

#include <mutex>
#include <vector>

namespace chrono {
namespace ros {

/// @addtogroup ros_robot_handlers
/// @{

/// Bidirectional subscriber that feeds waypoint targets from ROS into the
/// Chrono ViperWaypointFollower driver. Uses IPC to separate ROS symbols.
class CH_ROS_API ChROSViperWaypointFollowerHandler : public ChROSHandler {
  public:
    /// Constructor. Takes a ViperWaypointFollower driver
    ChROSViperWaypointFollowerHandler(double update_rate,
                                      std::shared_ptr<chrono::viper::ViperWaypointFollower> driver,
                                      const std::string& topic_name);

    /// Initializes the handler (validates topic; no ROS objects created here).
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

    /// Message type for IPC routing.
    virtual ipc::MessageType GetMessageType() const override { return ipc::MessageType::VIPER_WAYPOINT_FOLLOWER; }

    /// This handler receives incoming messages from ROS.
    virtual bool SupportsIncomingMessages() const override { return true; }

    /// Handle IPC messages carrying waypoint targets.
    virtual void HandleIncomingMessage(const ipc::Message& msg) override;

  protected:
    /// Send topic name once so subprocess can create the ROS subscriber.
    virtual std::vector<uint8_t> GetSerializedData(double time) override;

  private:
    /// Apply latest target to driver.
    void ApplyTarget(const ipc::ViperWaypointTarget& target);

  private:
    std::shared_ptr<chrono::viper::ViperWaypointFollower> m_driver;  ///< handle to the driver

    const std::string m_topic_name;          ///< name of the topic to subscribe to
    ipc::ViperWaypointTarget m_target;       ///< latest received target
    bool m_subscriber_setup_sent;            ///< tracks if setup message was sent

    std::mutex m_mutex;  ///< protects access to target
};

/// @} ros_robot_handlers

}  // namespace ros
}  // namespace chrono

#endif
