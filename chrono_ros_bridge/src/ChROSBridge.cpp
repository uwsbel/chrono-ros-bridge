#include <chrono>

#include "rclcpp/serialization.hpp"

#include "chrono_ros_bridge/ChROSBridge.h"

#include "chrono_ros_msgs/msg/ch_vehicle.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

using namespace rapidjson;
using namespace chrono::utils;
using namespace std::chrono_literals;

namespace chrono {
namespace ros {

ChROSBridge::ChROSBridge() : Node("chrono_ros_bridge") {
    auto port = this->declare_parameter("port", 50000);
    auto ip = this->declare_parameter("ip", "127.0.0.1");

    // TODO: Make a parameter to get port
    m_client = std::make_unique<ChSocketTCP>(port);
    m_client->connectToServer(ip, utils::ADDRESS);

    // Parsers
    m_message_parsers["ChSystem"] = std::bind(&ChROSBridge::MessageParser_ChSystem, this, std::placeholders::_1);
    m_message_parsers["ChVehicle"] = std::bind(&ChROSBridge::MessageParser_ChVehicle, this, std::placeholders::_1);
    m_message_parsers["ChCameraSensor"] =
        std::bind(&ChROSBridge::MessageParser_ChCameraSensor, this, std::placeholders::_1);

    m_timer = this->create_wall_timer(1us, std::bind(&ChROSBridge::TimerCallback, this));
}

void ChROSBridge::TimerCallback() {
    try {
        // -------
        // Receive
        // -------
        {
            std::string message;
            m_client->receiveMessage(message);

            // Parse the JSON string
            Document d;
            d.Parse(message.c_str());

            for (auto& m : d.GetArray()) {
                auto type = m["type"].GetString();
                if (m_message_parsers.count(type))
                    m_message_parsers[type](m);
            }
        }

        // ----
        // Send
        // ----
        {
            std::string message =
                "[{\"type\":\"ChDriverInputs\",\"data\":{\"throttle\":1.0,\"steering\":0.0,\"braking\":0.0}}]";
            m_client->sendMessage(message);
        }
    } catch (utils::ChExceptionSocket& exception) {
        std::cout << " ERRROR with socket system: \n" << exception.what() << std::endl;
    }
}

geometry_msgs::msg::Point ChROSBridge::MessageParser_Point(const Value& v) {
    auto arr = v.GetArray();

    geometry_msgs::msg::Point msg;
    msg.x = arr[0].GetDouble();
    msg.y = arr[1].GetDouble();
    msg.z = arr[2].GetDouble();
    return msg;
}

geometry_msgs::msg::Vector3 ChROSBridge::MessageParser_Vector3(const Value& v) {
    auto arr = v.GetArray();

    geometry_msgs::msg::Vector3 msg;
    msg.x = arr[0].GetDouble();
    msg.y = arr[1].GetDouble();
    msg.z = arr[2].GetDouble();
    return msg;
}

geometry_msgs::msg::Quaternion ChROSBridge::MessageParser_Quaternion(const Value& v) {
    auto arr = v.GetArray();

    geometry_msgs::msg::Quaternion msg;
    msg.x = arr[0].GetDouble();
    msg.y = arr[1].GetDouble();
    msg.z = arr[2].GetDouble();
    msg.w = arr[3].GetDouble();
    return msg;
}

void ChROSBridge::MessageParser_ChSystem(const Value& v) {
    auto type = v["type"].GetString();
    auto name = v["name"].GetString();
    auto data = v["data"].GetObject();

    rosgraph_msgs::msg::Clock msg;
    int64_t nanoseconds = (int64_t)(data["time"].GetDouble() * 1e9);
    msg.clock = rclcpp::Time(nanoseconds);
    Publish(name, type, "rosgraph_msgs/msg/Clock", msg);
}

void ChROSBridge::MessageParser_ChVehicle(const Value& v) {
    auto type = v["type"].GetString();
    auto name = v["name"].GetString();
    auto data = v["data"].GetObject();

    chrono_ros_msgs::msg::ChVehicle msg;
    msg.pose.position = MessageParser_Point(data["pos"]);
    msg.pose.orientation = MessageParser_Quaternion(data["rot"]);
    // msg.twist.linear = MessageParser_Vector3(data["vel"]);         // TODO
    // msg.twist.angular = MessageParser_Vector3(data["ang_vel"]);    // TODO
    // msg.accel.linear = MessageParser_Vector3(data["accel"]);       // TODO
    // msg.accel.angular = MessageParser_Vector3(data["ang_accel"]);  // TODO

    Publish(name, type, "chrono_ros_msgs/msg/ChVehicle", msg);
}

void ChROSBridge::MessageParser_ChCameraSensor(const Value& v) {
    auto type = v["type"].GetString();
    auto name = v["name"].GetString();
    auto data = v["data"].GetObject();

    sensor_msgs::msg::Image msg;
    msg.width = data["width"].GetUint64();
    msg.height = data["height"].GetUint64();
    msg.step = msg.width * data["size"].GetUint64();
    msg.encoding = "rgba8";

    const char* image_ptr = data["image"].GetString();
    msg.data = std::vector<uint8_t>(image_ptr, image_ptr + msg.height * msg.step);

    Publish(name, type, "sensor_msgs/msg/Image", msg);
}

}  // namespace ros
}  // namespace chrono

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<chrono::ros::ChROSBridge>());
    rclcpp::shutdown();
    return 0;
}
