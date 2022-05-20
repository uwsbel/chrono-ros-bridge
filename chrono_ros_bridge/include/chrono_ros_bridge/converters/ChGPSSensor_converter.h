#ifndef CH_GPS_SENSOR_CONVERTER_H
#define CH_GPS_SENSOR_CONVERTER_H

#include <string>
#include <vector>

#include "chrono_ros_bridge/common.h"

#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace chrono {
namespace ros {

/**
 * Converter _from_ json _to_ a ROS message.
 *
 * In the case of ChGPSSensor, the message structure will look like the following:
 * {
 * 	type: ChGPSSensor
 * 	id: gps
 * 	data: {
 * 		latitude: <latitude>
 * 		longitude: <longitude>
 * 		altitude: <altitude>
 * 	}
 * }
 */
SerializedMessagePtr ChGPSSensor_converter(chrono::utils::ChJSONReader& reader) {
    std::string type, id;
    reader >> type >> id >> reader.GetObject();

    std::string latitude, longitude, altitude;

    sensor_msgs::msg::NavSatFix msg;
    reader >> latitude >> longitude >> altitude;

    msg.latitude = std::stod(latitude);
    msg.longitude = std::stod(longitude);
    msg.altitude = std::stod(altitude);

    return Serialize(msg);
}

}  // namespace ros
}  // namespace chrono

#endif

