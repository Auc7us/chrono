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

#include "chrono_ros/handlers/robot/viper/ChROSViperSpeedDriverHandler.h"

#include "chrono_ros/handlers/ChROSHandlerUtilities.h"
#include "chrono_ros/ipc/ChROSIPCMessage.h"

#include <iostream>

using namespace chrono::viper;

namespace chrono {
namespace ros {

ChROSViperSpeedDriverHandler::ChROSViperSpeedDriverHandler(double update_rate,
                                                           std::shared_ptr<ViperSpeedDriver> driver,
                                                           const std::string& topic_name)
    : ChROSHandler(update_rate),
      m_driver(driver),
      m_topic_name(topic_name),
      m_command{0.0},
      m_subscriber_setup_sent(false) {}

bool ChROSViperSpeedDriverHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, m_topic_name)) {
        return false;
    }
    return true;
}

std::vector<uint8_t> ChROSViperSpeedDriverHandler::GetSerializedData(double time) {
    if (!m_subscriber_setup_sent) {
        m_subscriber_setup_sent = true;
        std::vector<uint8_t> data(m_topic_name.begin(), m_topic_name.end());
        return data;
    }

    return {};
}

void ChROSViperSpeedDriverHandler::HandleIncomingMessage(const ipc::Message& msg) {
    if (msg.header.payload_size != sizeof(ipc::ViperSpeedCommand)) {
        std::cerr << "ViperSpeedDriverHandler: Received payload size mismatch" << std::endl;
        return;
    }

    const auto* data = msg.GetPayload<ipc::ViperSpeedCommand>();
    ApplySpeed(data->speed);
}

void ChROSViperSpeedDriverHandler::ApplySpeed(double speed) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_command.speed = speed;
    m_driver->SetSpeed(m_command.speed);
}

}  // namespace ros
}  // namespace chrono
