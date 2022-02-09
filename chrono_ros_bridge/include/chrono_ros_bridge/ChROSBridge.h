#include <memory>
#include <functional>
#include <mutex>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include "chrono_ros_bridge/ChSocket.h"
#include "chrono_ros_msgs/msg/ch_driver_inputs.hpp"
#include "chrono_ros_msgs/msg/ch_vehicle.hpp"
#include "chrono_ros_msgs/msg/ch_driver_inputs.hpp"

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
    template <typename MsgType>
    void CreateSubscription(const std::string& type,
                            const std::string& topic,
                            std::function<void(std::shared_ptr<MsgType>)>& callback,
                            rclcpp::QoS qos = 1) {
        auto _callback = [&](std::shared_ptr<rclcpp::SerializedMessage> msg) {
            std::scoped_lock lock(m_mutex);

            m_writer.StartObject();

            // type
            {
                m_writer.Key("type");
                m_writer.String(type.c_str());
            }

            // name
            {
                m_writer.Key("name");
                m_writer.String(topic.c_str());
            }

            // data
            {
                m_writer.Key("data");
                m_writer.StartObject();
                callback(DeserializeMessage<MsgType>(msg));
                m_writer.EndObject();
            }
            m_writer.EndObject();
        };
        m_subscribers[type] =
            this->create_generic_subscription(topic, rosidl_generator_traits::name<MsgType>(), qos, _callback);
    }

    template <typename MsgType>
    std::shared_ptr<MsgType> DeserializeMessage(const std::shared_ptr<rclcpp::SerializedMessage> msg) {
        static rclcpp::Serialization<MsgType> serializer;

        auto deserialized_msg = std::make_shared<MsgType>();
        serializer.deserialize_message(msg.get(), deserialized_msg.get());
        return deserialized_msg;
    }

    void MessageGenerator_ChDriverInputs(std::shared_ptr<chrono_ros_msgs::msg::ChDriverInputs> msg);

    // ---------------
    // Message Parsers
    // ---------------
    template <typename MsgType>
    void Publish(const std::string& name, const std::string& type, const MsgType& msg, const rclcpp::QoS& qos = 1) {
        auto& publisher = m_publishers[type];
        if (not publisher)
            publisher = this->create_generic_publisher(name, rosidl_generator_traits::name<MsgType>(), qos);

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

    std::mutex m_mutex;
    rapidjson::StringBuffer m_buffer;
    rapidjson::Writer<rapidjson::StringBuffer> m_writer;
    std::map<std::string, std::shared_ptr<rclcpp::GenericSubscription>> m_subscribers;

    std::map<std::string, std::function<void(const rapidjson::Value& v)>> m_message_parsers;
    std::map<std::string, std::shared_ptr<rclcpp::GenericPublisher>> m_publishers;
};

}  // namespace ros
}  // namespace chrono
