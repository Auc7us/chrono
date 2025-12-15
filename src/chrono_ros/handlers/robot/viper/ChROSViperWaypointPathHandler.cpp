#include "chrono_ros/handlers/robot/viper/ChROSViperWaypointPathHandler.h"

#include "chrono_ros/ipc/ChROSIPCMessage.h"

#include <cstring>

using namespace chrono::viper;

namespace chrono {
namespace ros {

ChROSViperWaypointPathHandler::ChROSViperWaypointPathHandler(
    double update_rate,
    std::shared_ptr<chrono::viper::ViperWaypointFollower> driver,
    const std::string& topic_name,
    const std::string& frame_id)
    : ChROSHandler(update_rate),
      m_driver(driver),
      m_topic_name(topic_name),
      m_frame_id(frame_id),
      m_last_publish_time(-1.0) {}

bool ChROSViperWaypointPathHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, m_topic_name)) {
        return false;
    }
    return true;
}

std::vector<uint8_t> ChROSViperWaypointPathHandler::GetSerializedData(double time) {
    // Throttle based on requested update rate
    double frame_time = GetUpdateRate() == 0 ? 0.0 : 1.0 / GetUpdateRate();
    if (m_last_publish_time >= 0.0 && frame_time > 0.0 && (time - m_last_publish_time) < frame_time) {
        return {};
    }

    SerializePath(time);
    m_last_publish_time = time;
    return m_buffer;
}

void ChROSViperWaypointPathHandler::SerializePath(double time) {
    if (!m_driver) {
        m_buffer.clear();
        return;
    }

    const auto path_points = m_driver->GetPathPoints();

    ipc::ViperWaypointPathHeader header{};
    std::memset(&header, 0, sizeof(header));
    std::strncpy(header.topic_name, m_topic_name.c_str(), sizeof(header.topic_name) - 1);
    std::strncpy(header.frame_id, m_frame_id.c_str(), sizeof(header.frame_id) - 1);
    header.point_count = static_cast<uint32_t>(path_points.size());

    // Compute total size and guard against oversized payload
    const size_t points_bytes = static_cast<size_t>(header.point_count) * sizeof(ipc::ViperWaypointPathPoint);
    const size_t total_size = sizeof(ipc::ViperWaypointPathHeader) + points_bytes;

    if (total_size > ipc::MAX_PAYLOAD_SIZE) {
        // Clamp the number of points to fit inside MAX_PAYLOAD_SIZE
        header.point_count = static_cast<uint32_t>(
            (ipc::MAX_PAYLOAD_SIZE - sizeof(ipc::ViperWaypointPathHeader)) / sizeof(ipc::ViperWaypointPathPoint));
    }

    const size_t clamped_points_bytes =
        static_cast<size_t>(header.point_count) * sizeof(ipc::ViperWaypointPathPoint);
    const size_t clamped_total_size = sizeof(ipc::ViperWaypointPathHeader) + clamped_points_bytes;

    m_buffer.resize(clamped_total_size);

    // Copy header
    std::memcpy(m_buffer.data(), &header, sizeof(header));

    // Copy points
    auto* dst_points = reinterpret_cast<ipc::ViperWaypointPathPoint*>(m_buffer.data() + sizeof(header));
    for (uint32_t i = 0; i < header.point_count; ++i) {
        const auto& pt = path_points[i];
        dst_points[i].x = pt.x();
        dst_points[i].y = pt.y();
        dst_points[i].z = pt.z();
    }
}

}  // namespace ros
}  // namespace chrono

