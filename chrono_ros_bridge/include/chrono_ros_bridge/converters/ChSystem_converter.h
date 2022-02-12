#ifndef CH_SYSTEM_CONVERTER_H
#define CH_SYSTEM_CONVERTER_H

#include <string>

#include "chrono_ros_bridge/common.h"

#include "rosgraph_msgs/msg/clock.hpp"

namespace chrono {
namespace ros {

/**
 * Converter _from_ json _to_ a ROS message.
 *
 * In the case of ChSystem, the message structure will look like the following:
 * {
 * 	type: ChSystem
 * 	name: clock
 * 	data: {
 * 		time: <time>
 * 	}
 * }
 *
 * The /clock topic used by ROS to distribute the time to ROS nodes. If the parameter /use_sim_time
 * is set to true, the /clock topic will be read by ROS and used by each node as the ground truth. This
 * means the stack is essentially then synchronized with the simulation.
 */
SerializedMessagePtr ChSystem_converter(chrono::utils::ChJSONReader& reader) {
    std::string type, id;
    reader >> type >> id >> reader.GetObject();

    double time;
    reader >> time;

    rosgraph_msgs::msg::Clock msg;
    int64_t nanoseconds = (int64_t)(time * 1e9);
    msg.clock = rclcpp::Time(nanoseconds);

    return Serialize(msg);
}

}  // namespace ros
}  // namespace chrono

#endif

