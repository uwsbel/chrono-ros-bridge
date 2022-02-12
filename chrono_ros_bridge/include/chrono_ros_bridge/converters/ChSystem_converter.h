#ifndef CH_SYSTEM_CONVERTER_H
#define CH_SYSTEM_CONVERTER_H

#include "chrono_ros_bridge/converters/common.h"

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
void ChSystem_converter(const rapidjson::Value& v, ChROSBridge* bridge);

}  // namespace ros
}  // namespace chrono

#endif
