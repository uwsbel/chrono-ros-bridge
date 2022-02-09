#include <memory>
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include "chrono_ros_bridge/ChSocket.h"

namespace chrono {
namespace ros {

class ChROSBridge : public rclcpp::Node {
  public:
    ChROSBridge();

  private:
    void TimerCallback();

    // ------------------
    // Message Generators
    // ------------------

    // void MessageGenerator_ChDriverInputs(rapidjson::Writer<rapidjson::StringBuffer>& writer);

    // ---------------
    // Message Parsers
    // ---------------
    template <typename MsgType>
    void Publish(const std::string& name,
                 const std::string& map_type,
                 const std::string& msg_type,
                 const MsgType& msg,
                 const rclcpp::QoS& qos = 1) {
        auto& publisher = m_publishers[map_type];
        if (not publisher)
            publisher = this->create_generic_publisher(name, msg_type, qos);

        static rclcpp::Serialization<MsgType> serializer;
        static rclcpp::SerializedMessage serialized_msg;

        serializer.serialize_message(&msg, &serialized_msg);

        publisher->publish(serialized_msg);
    }

    geometry_msgs::msg::Point MessageParser_Point(const rapidjson::Value& v);
    geometry_msgs::msg::Vector3 MessageParser_Vector3(const rapidjson::Value& v);
    geometry_msgs::msg::Quaternion MessageParser_Quaternion(const rapidjson::Value& v);

    void MessageParser_ChSystem(const rapidjson::Value& v);
    void MessageParser_ChVehicle(const rapidjson::Value& v);
    void MessageParser_ChCameraSensor(const rapidjson::Value& v);

    // -----
    // Other
    // -----

    rclcpp::TimerBase::SharedPtr m_timer;

    std::unique_ptr<chrono::utils::ChSocketTCP> m_client;

    std::map<std::string, std::function<void(rapidjson::Writer<rapidjson::StringBuffer>& writer)>> m_message_generators;
    std::map<std::string, std::shared_ptr<rclcpp::GenericSubscription>> m_subscribers;

    std::map<std::string, std::function<void(const rapidjson::Value& v)>> m_message_parsers;
    std::map<std::string, std::shared_ptr<rclcpp::GenericPublisher>> m_publishers;
};

}  // namespace ros
}  // namespace chrono
