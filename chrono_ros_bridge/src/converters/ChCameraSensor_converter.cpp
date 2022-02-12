#include "chrono_ros_bridge/converters/ChCameraSensor_converter.h"

#include "sensor_msgs/msg/image.hpp"

#include "chrono_ros_bridge/ChROSBridge.h"

namespace chrono {
namespace ros {

void ChCameraSensor_converter(const rapidjson::Value& v, ChROSBridge* bridge) {
    auto type = v["type"].GetString();
    auto name = v["name"].GetString();
    auto data = v["data"].GetObject();

    sensor_msgs::msg::Image msg;
    msg.width = data["width"].GetUint64();
    msg.height = data["height"].GetUint64();
    msg.step = msg.width * data["size"].GetUint64();
    msg.encoding = "rgba8";

    const char* image_ptr = data["image"].GetString();
    msg.data = std::vector<uint8_t>(image_ptr, image_ptr + msg.height * msg.step);

    bridge->Publish(name, type, msg);
}

CH_REGISTER_MESSAGE_PARSER("ChCameraSensor", ChCameraSensor_converter)

}  // namespace ros
}  // namespace chrono

