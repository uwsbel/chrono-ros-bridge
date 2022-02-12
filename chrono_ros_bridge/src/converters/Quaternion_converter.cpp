#include "chrono_ros_bridge/converters/Quaternion_converter.h"

namespace chrono {
namespace ros {

geometry_msgs::msg::Quaternion Quaternion_converter(const rapidjson::Value& v) {
    auto arr = v.GetArray();

    geometry_msgs::msg::Quaternion msg;
    msg.x = arr[0].GetDouble();
    msg.y = arr[1].GetDouble();
    msg.z = arr[2].GetDouble();
    msg.w = arr[3].GetDouble();
    return msg;
}

}  // namespace ros
}  // namespace chrono

