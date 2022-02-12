#ifndef CH_VEHICLE_CONVERTER_H
#define CH_VEHICLE_CONVERTER_H

#include <string>
#include <array>

#include "chrono_ros_bridge/common.h"

#include "chrono_ros_msgs/msg/ch_vehicle.hpp"

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
SerializedMessagePtr ChVehicle_converter(chrono::utils::ChJSONReader& reader) {
    std::string type, id;
    reader >> type >> id >> reader.GetObject();

    std::array<double, 3> pos, lin_vel, ang_vel, lin_acc, ang_acc;
    std::array<double, 4> rot;
    reader >> pos >> rot >> lin_vel >> ang_vel >> lin_acc >> ang_acc;

    chrono_ros_msgs::msg::ChVehicle msg;

    msg.pose.position.x = pos[0];
    msg.pose.position.y = pos[1];
    msg.pose.position.z = pos[2];

    msg.pose.orientation.x = rot[0];
    msg.pose.orientation.y = rot[1];
    msg.pose.orientation.z = rot[2];
    msg.pose.orientation.z = rot[3];

    msg.twist.linear.x = lin_vel[0];
    msg.twist.linear.y = lin_vel[1];
    msg.twist.linear.z = lin_vel[2];

    msg.twist.angular.x = ang_vel[0];
    msg.twist.angular.y = ang_vel[1];
    msg.twist.angular.z = ang_vel[2];

    msg.accel.linear.x = lin_acc[0];
    msg.accel.linear.y = lin_acc[1];
    msg.accel.linear.z = lin_acc[2];

    msg.accel.angular.x = ang_acc[0];
    msg.accel.angular.y = ang_acc[1];
    msg.accel.angular.z = ang_acc[2];

    return Serialize(msg);
}

}  // namespace ros
}  // namespace chrono

#endif

