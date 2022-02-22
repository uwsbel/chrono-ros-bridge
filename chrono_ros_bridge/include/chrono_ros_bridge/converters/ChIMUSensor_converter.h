#ifndef CH_IMU_SENSOR_CONVERTERS_H
#define CH_IMU_SENSOR_CONVERTERS_H

#include <string>
#include <vector>

#include "chrono_ros_bridge/common.h"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

namespace chrono {
namespace ros {

/**
 * Converter _from_ json _to_ a ROS message.
 *
 * In the case of ChIMUSensor, three total messages may be received:
 *	ChAccelerometerSensor
 *	ChGyroscopeSensor
 *	ChMagnetometerSensor
 *
 * This parser is for the ChAccelerometerSensor, which has the following message structure:
 * {
 * 	type: ChAccelerometerSensor
 * 	id: accelerometer
 * 	data: {
 * 		X: <x>
 * 		Y: <y>
 * 		Z: <z>
 * 	}
 * }
 */
SerializedMessagePtr ChAccelerometerSensor_converter(chrono::utils::ChJSONReader& reader) {
    std::string type, id;
    reader >> type >> id >> reader.GetObject();

    sensor_msgs::msg::Imu msg;
    reader >> msg.linear_acceleration.x >> msg.linear_acceleration.y >> msg.linear_acceleration.z;

    return Serialize(msg);
}

/**
 * Converter _from_ json _to_ a ROS message.
 *
 * In the case of ChIMUSensor, three total messages may be received:
 *	ChAccelerometerSensor
 *	ChGyroscopeSensor
 *	ChMagnetometerSensor
 *
 * This parser is for the ChGyroscopeSensor, which has the following message structure:
 * {
 * 	type: ChGyroscopeSensor
 * 	id: gyroscope
 * 	data: {
 * 		Roll: <roll>
 * 		Pitch: <pitch>
 * 		Yaw: <yaw>
 * 	}
 * }
 */
SerializedMessagePtr ChGyroscopeSensor_converter(chrono::utils::ChJSONReader& reader) {
    std::string type, id;
    reader >> type >> id >> reader.GetObject();

    sensor_msgs::msg::Imu msg;
    reader >> msg.angular_velocity.x >> msg.angular_velocity.y >> msg.angular_velocity.z;

    return Serialize(msg);
}

/**
 * Converter _from_ json _to_ a ROS message.
 *
 * In the case of ChIMUSensor, three total messages may be received:
 *	ChAccelerometerSensor
 *	ChGyroscopeSensor
 *	ChMagnetometerSensor
 *
 * This parser is for the ChMagnetometerSensor, which has the following message structure:
 * {
 * 	type: ChMagnetometerSensor
 * 	id: magnetometer
 * 	data: {
 * 		X: <x>
 * 		Y: <y>
 * 		Z: <z>
 * 	}
 * }
 */
SerializedMessagePtr ChMagnetometerSensor_converter(chrono::utils::ChJSONReader& reader) {
    std::string type, id;
    reader >> type >> id >> reader.GetObject();

    sensor_msgs::msg::MagneticField msg;
    reader >> msg.magnetic_field.x >> msg.magnetic_field.y >> msg.magnetic_field.z;

    return Serialize(msg);
}

}  // namespace ros
}  // namespace chrono

#endif

