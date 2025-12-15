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

#include "chrono_ros/handlers/robot/viper/ChROSViperWaypointFollowerHandler.h"

#include "chrono_ros/handlers/ChROSHandlerUtilities.h"
#include "chrono_ros/ipc/ChROSIPCMessage.h"

#include <iostream>

using namespace chrono::viper;

namespace chrono {
namespace ros {

ChROSViperWaypointFollowerHandler::ChROSViperWaypointFollowerHandler(double update_rate,
                                                                     std::shared_ptr<ViperWaypointFollower> driver,
                                                                     const std::string& topic_name)
    : ChROSHandler(update_rate),
      m_driver(driver),
      m_topic_name(topic_name),
      m_target{-5.0, 0.0, 0.0},
      m_subscriber_setup_sent(false) {}

bool ChROSViperWaypointFollowerHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, m_topic_name)) {
        return false;
    }
    return true;
}

std::vector<uint8_t> ChROSViperWaypointFollowerHandler::GetSerializedData(double time) {
    if (!m_subscriber_setup_sent) {
        m_subscriber_setup_sent = true;
        std::vector<uint8_t> data(m_topic_name.begin(), m_topic_name.end());
        return data;
    }
    return {};
}

void ChROSViperWaypointFollowerHandler::HandleIncomingMessage(const ipc::Message& msg) {
    if (msg.header.payload_size != sizeof(ipc::ViperWaypointTarget)) {
        std::cerr << "ViperWaypointFollowerHandler: Received payload size mismatch" << std::endl;
        return;
    }

    const auto* data = msg.GetPayload<ipc::ViperWaypointTarget>();
    ApplyTarget(*data);
}

void ChROSViperWaypointFollowerHandler::ApplyTarget(const ipc::ViperWaypointTarget& target) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_target = target;
    m_driver->SetTarget(m_target.x, m_target.y, m_target.z);
}

}  // namespace ros
}  // namespace chrono
