#include "chrono_ros_bridge/converters/Vector_converter.h"

namespace chrono {
namespace ros {

template <typename VectorT>
VectorT Vector_converter(const rapidjson::Value& v) {
    auto arr = v.GetArray();

    VectorT msg;
    msg.x = arr[0].GetDouble();
    msg.y = arr[1].GetDouble();
    msg.z = arr[2].GetDouble();
    return msg;
}

template geometry_msgs::msg::Point Vector_converter<geometry_msgs::msg::Point>(const rapidjson::Value& v);
template geometry_msgs::msg::Vector3 Vector_converter<geometry_msgs::msg::Vector3>(const rapidjson::Value& v);

}  // namespace ros
}  // namespace chrono
