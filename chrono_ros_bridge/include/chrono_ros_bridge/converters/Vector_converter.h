#ifndef VECTOR_CONVERTER_H
#define VECTOR_CONVERTER_H

#include "chrono_ros_bridge/converters/common.h"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace chrono {
namespace ros {

/**
 * Converter _from_ json _to_ a ROS message.
 *
 * Converts an array of 3 doubles to a ROS message with x,y,z attributes. Will return it.
 */
template <typename VectorT>
VectorT Vector_converter(const rapidjson::Value& v);

}  // namespace ros
}  // namespace chrono

#endif

