#ifndef CH_CAMERA_SENSOR_CONVERTER_H
#define CH_CAMERA_SENSOR_CONVERTER_H

#include "chrono_ros_bridge/converters/common.h"

namespace chrono {
namespace ros {

/**
 * Converter _from_ json _to_ a ROS message.
 *
 * In the case of ChCameraSensor, the message structure will look like the following:
 * {
 * 	type: ChCameraSensor
 * 	name: camera
 * 	data: {
 * 		width: <width>
 * 		height: <height>
 * 		size: <size>
 * 		image: <image> # the image is formated as a string
 * 	}
 * }
 *
 * The ChCameraSensor will generate images using raytracing. The message will contain the width,
 * height, and step size (size), as well as the image itself encoded in a string. The image will be
 * structured in accordance with Chrono::Sensor, where it's structured in row major format.
 *
 * TODO: Currently it is assumed the image encoding is rgba8
 */
void ChCameraSensor_converter(const rapidjson::Value& v, ChROSBridge* bridge);

}  // namespace ros
}  // namespace chrono

#endif
