#ifndef CH_VIPER_WAYPOINT_PATH_HANDLER_H
#define CH_VIPER_WAYPOINT_PATH_HANDLER_H

#include "chrono_ros/ChROSHandler.h"
#include "chrono_ros/handlers/ChROSHandlerUtilities.h"
#include "chrono_ros/handlers/robot/viper/ChROSViperWaypointPathHandler_ipc.h"

#include "chrono_models/robot/viper/Viper.h"

#include <vector>

namespace chrono {
namespace ros {

/// Publisher handler that serializes the Viper waypoint follower's path into an
/// IPC buffer and publishes nav_msgs/Path from the ROS subprocess.
class CH_ROS_API ChROSViperWaypointPathHandler : public ChROSHandler {
  public:
    ChROSViperWaypointPathHandler(double update_rate,
                                  std::shared_ptr<chrono::viper::ViperWaypointFollower> driver,
                                  const std::string& topic_name,
                                  const std::string& frame_id = "map");

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;
    virtual ipc::MessageType GetMessageType() const override { return ipc::MessageType::VIPER_WAYPOINT_PATH; }

    /// Serialize path data for IPC transmission.
    virtual std::vector<uint8_t> GetSerializedData(double time) override;

  private:
    /// Serialize the current path into m_buffer.
    void SerializePath(double time);

    std::shared_ptr<chrono::viper::ViperWaypointFollower> m_driver;
    std::string m_topic_name;
    std::string m_frame_id;

    double m_last_publish_time;
    std::vector<uint8_t> m_buffer;
};

}  // namespace ros
}  // namespace chrono

#endif
