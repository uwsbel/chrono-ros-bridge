#ifndef CH_DRIVER_INPUTS_CONVERTER_H
#define CH_DRIVER_INPUTS_CONVERTER_H

#include "chrono_ros_bridge/common.h"

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
void ChDriverInputs_converter(SerializedMessagePtr serialized_msg, chrono::utils::ChJSONWriter& writer) {
    auto msg = Deserialize<chrono_ros_msgs::msg::ChDriverInputs>(serialized_msg);

    writer.Key("steering") << msg.steering;
    writer.Key("throttle") << msg.throttle;
    writer.Key("braking") << msg.braking;
}

}  // namespace ros
}  // namespace chrono

#endif

