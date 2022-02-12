#include "chrono_ros_bridge/converters/ChVehicle_converter.h"

#include "chrono_ros_bridge/converters/Vector_converter.h"
#include "chrono_ros_bridge/converters/Quaternion_converter.h"

#include "chrono_ros_msgs/msg/ch_vehicle.hpp"

#include "chrono_ros_bridge/ChROSBridge.h"

namespace chrono {
namespace ros {

void ChVehicle_converter(const rapidjson::Value& v, ChROSBridge* bridge) {
    auto type = v["type"].GetString();
    auto name = v["name"].GetString();
    auto data = v["data"].GetObject();

    chrono_ros_msgs::msg::ChVehicle msg;
    msg.pose.position = Vector_converter<geometry_msgs::msg::Point>(data["pos"]);
    msg.pose.orientation = Quaternion_converter(data["rot"]);
    msg.twist.linear = Vector_converter<geometry_msgs::msg::Vector3>(data["lin_vel"]);
    msg.twist.angular = Vector_converter<geometry_msgs::msg::Vector3>(data["ang_vel"]);
    msg.accel.linear = Vector_converter<geometry_msgs::msg::Vector3>(data["lin_acc"]);
    msg.accel.angular = Vector_converter<geometry_msgs::msg::Vector3>(data["ang_acc"]);

    bridge->Publish(name, type, msg);
}

CH_REGISTER_MESSAGE_PARSER("ChVehicle", ChVehicle_converter)

}  // namespace ros
}  // namespace chrono

