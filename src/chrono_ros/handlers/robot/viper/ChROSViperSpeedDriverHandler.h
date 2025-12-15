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
// Authors: Keshav Sharan
// =============================================================================
//
// Handler responsible for receiving and updating the DC motor control commands
// for the Viper rover
//
// =============================================================================

#ifndef CH_VIPER_SPEED_DRIVER_HANDLER_H
#define CH_VIPER_SPEED_DRIVER_HANDLER_H

#include "chrono_ros/ChROSHandler.h"
#include "chrono_models/robot/viper/Viper.h"
#include "chrono_ros/handlers/robot/viper/ChROSViperSpeedDriverHandler_ipc.h"

#include <mutex>
#include <vector>

namespace chrono {
namespace ros {

/// @addtogroup ros_robot_handlers
/// @{

/// Bidirectional subscriber handler that applies wheel speed commands coming
/// from ROS (Float32 topic) to a Chrono ViperSpeedDriver. Uses IPC to avoid ROS
/// symbols in the main process.
class CH_ROS_API ChROSViperSpeedDriverHandler : public ChROSHandler {
  public:
    /// Constructor. Takes a ViperSpeedDriver instance.
    ChROSViperSpeedDriverHandler(double update_rate,
                                 std::shared_ptr<chrono::viper::ViperSpeedDriver> driver,
                                 const std::string& topic_name);

    /// Initializes the handler (validates topic name; no ROS objects created in main process).
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

    /// Message type for IPC routing.
    virtual ipc::MessageType GetMessageType() const override { return ipc::MessageType::VIPER_SPEED_DRIVER; }

    /// This handler receives incoming messages from the ROS subprocess.
    virtual bool SupportsIncomingMessages() const override { return true; }

    /// Handle IPC messages carrying speed commands from ROS.
    virtual void HandleIncomingMessage(const ipc::Message& msg) override;

  protected:
    /// Send topic name once so the subprocess can create the ROS subscriber.
    virtual std::vector<uint8_t> GetSerializedData(double time) override;

  private:
    /// Apply the latest command to the driver.
    void ApplySpeed(double speed);

  private:
    std::shared_ptr<chrono::viper::ViperSpeedDriver> m_driver;  ///< Handle to the driver

    const std::string m_topic_name;  ///< Topic name for speed commands
    ipc::ViperSpeedCommand m_command;  ///< Latest received speed command
    bool m_subscriber_setup_sent;      ///< Tracks if setup message sent to subprocess

    std::mutex m_mutex;
};

/// @} ros_robot_handlers

}  // namespace ros
}  // namespace chrono

#endif  // CH_VIPER_SPEED_DRIVER_HANDLER_H
