#ifndef QUATERNION_CONVERTER_H
#define QUATERNION_CONVERTER_H

#include "chrono_ros_bridge/converters/common.h"

#include <geometry_msgs/msg/quaternion.hpp>

namespace chrono {
namespace ros {

/**
 * Converter _from_ json _to_ a ROS message.
 *
 * Converts an array of 4 doubles to a geometry::msg::Quaternion. Will return it.
 */
geometry_msgs::msg::Quaternion Quaternion_converter(const rapidjson::Value& v);

}  // namespace ros
}  // namespace chrono

#endif
