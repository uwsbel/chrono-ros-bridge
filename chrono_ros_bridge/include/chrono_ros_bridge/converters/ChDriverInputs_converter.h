#ifndef CH_DRIVER_INPUTS_CONVERTER_H
#define CH_DRIVER_INPUTS_CONVERTER_H

#include "chrono_ros_bridge/converters/common.h"

#include "chrono_ros_msgs/msg/ch_driver_inputs.hpp"

namespace chrono {
namespace ros {

/**
 * Converter _from_ json _to_ a ROS message.
 *
 * In the case of ChDriverInputs, the message structure will look like the following:
 * {
 * 	type: ChDriverInputs
 * 	name: inputs
 * 	data: {
 * 		steering: <steering>
 * 		throttle: <throttle>
 * 		braking: <braking>
 * 	}
 * }
 *
 * All values are Doubles.
 */
void ChDriverInputs_converter(const chrono_ros_msgs::msg::ChDriverInputs& msg,
                              rapidjson::Writer<rapidjson::StringBuffer>& writer);

}  // namespace ros
}  // namespace chrono

#endif

