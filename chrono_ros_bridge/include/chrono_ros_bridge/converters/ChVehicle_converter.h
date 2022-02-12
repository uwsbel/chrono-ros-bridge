#ifndef CH_VEHICLE_CONVERTER_H
#define CH_VEHICLE_CONVERTER_H

#include "chrono_ros_bridge/converters/common.h"

namespace chrono {
namespace ros {

/**
 * Converter _from_ json _to_ a ROS message.
 *
 * In the case of ChVehicle, the message structure will look like the following:
 * {
 * 	type: ChVehicle
 * 	name: vehicle
 * 	data: {
 * 		pos: [x, y, z]
 * 		rot: [x, y, z, w]
 * 		lin_vel: [x, y, z]
 * 		ang_vel: [x, y, z]
 * 		lin_acc: [x, y, z]
 * 		ang_acc: [x, y, z]
 * 	}
 * }
 *
 * This message is meant to help the stack "cheat". In essence, it allows control stacks
 * to grab the position of the vehicle without the need for estimation or maybe even perception.
 */
void ChVehicle_converter(const rapidjson::Value& v, ChROSBridge* bridge);

}  // namespace ros
}  // namespace chrono

#endif
