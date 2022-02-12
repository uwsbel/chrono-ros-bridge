#include "chrono_ros_bridge/converters/ChSystem_converter.h"

#include "rosgraph_msgs/msg/clock.hpp"

#include "chrono_ros_bridge/ChROSBridge.h"

namespace chrono {
namespace ros {

void ChSystem_converter(const rapidjson::Value& v, ChROSBridge* bridge) {
    auto type = v["type"].GetString();
    auto name = v["name"].GetString();
    auto data = v["data"].GetObject();

    rosgraph_msgs::msg::Clock msg;
    int64_t nanoseconds = (int64_t)(data["time"].GetDouble() * 1e9);
    msg.clock = rclcpp::Time(nanoseconds);
    bridge->Publish(name, type, msg);
}

CH_REGISTER_MESSAGE_PARSER("ChSystem", ChSystem_converter)

}  // namespace ros
}  // namespace chrono
