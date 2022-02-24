#ifndef CH_CAMERA_SENSOR_CONVERTER_H
#define CH_CAMERA_SENSOR_CONVERTER_H

#include <string>
#include <vector>

#include "chrono_ros_bridge/common.h"

#include "sensor_msgs/msg/image.hpp"

namespace chrono {
namespace ros {

/**
 * Converter _from_ json _to_ a ROS message.
 *
 * In the case of ChCameraSensor, the message structure will look like the following:
 * {
 * 	type: ChCameraSensor
 * 	id: camera
 * 	data: {
 * 		width: <width>
 * 		height: <height>
 * 		size: <size>
 * 		encoding: <encoding>
 * 		image: <image> # the image is formated as a string
 * 	}
 * }
 *
 * The ChCameraSensor will generate images using raytracing. The message will contain the width,
 * height, and step size (size), as well as the image itself encoded in a string. The image will be
 * structured in accordance with Chrono::Sensor, where it's structured in row major format.
 */
SerializedMessagePtr ChCameraSensor_converter(chrono::utils::ChJSONReader& reader) {
    std::string type, id;
    reader >> type >> id >> reader.GetObject();

    sensor_msgs::msg::Image msg;
    reader >> msg.width >> msg.height >> msg.step >> msg.encoding >> msg.data;
    msg.step *= msg.width;

    msg.header.frame_id = "map";

    return Serialize(msg);
}

}  // namespace ros
}  // namespace chrono

#endif
