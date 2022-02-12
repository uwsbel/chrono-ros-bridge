#ifndef CH_ROS_BRIDGE_H
#define CH_ROS_BRIDGE_H

#include <rclcpp/rclcpp.hpp>

#include "chrono_ros_bridge/ChSocket.h"
#include "chrono_ros_bridge/common.h"

namespace chrono {
namespace ros {

struct Topic {
    std::string id;
    std::string type;

    bool operator()(const Topic& a, const Topic& b) const { return a.id + a.type > b.id + b.type; }
};

class ChROSBridge : public rclcpp::Node {
  public:
    ChROSBridge();

    template <typename MessageT>
    void AddMessageParser(const std::string& type, Json2RosConverter converter) {
        m_message_parsers[type] = converter;
        m_topic_types[type] = rosidl_generator_traits::name<MessageT>();
    }

    template <typename MessageT>
    void AddMessageGenerator(const Topic topic, SerializedMessageCallback callback) {
        auto callback_wrapper = [&](SerializedMessagePtr msg) {
            m_writer.StartObject(topic.type);
            callback(msg, m_writer);
            m_writer.EndObject();
            m_writer.EndObject();
        };

        m_topic_types[topic.type] = rosidl_generator_traits::name<MessageT>();

        // TODO: Currently only support QoS of 1
        auto subscriber = this->create_generic_subscription(topic.id, m_topic_types[topic.type], 1, callback_wrapper);
        m_subscribers.push_back(subscriber);
    }

  private:
    void TimerCallback();

    void Publish(const Topic topic, SerializedMessagePtr msg, const rclcpp::QoS& qos = 1);

    // -----
    // Other
    // -----

    rclcpp::TimerBase::SharedPtr m_timer;

    std::unique_ptr<chrono::utils::ChSocketTCP> m_client;

    chrono::utils::ChJSONWriter m_writer;
    std::vector<std::shared_ptr<rclcpp::GenericSubscription>> m_subscribers;

    chrono::utils::ChJSONReader m_reader;
    std::map<std::string, std::string> m_topic_types;
    std::map<std::string, Json2RosConverter> m_message_parsers;
    std::map<Topic, std::shared_ptr<rclcpp::GenericPublisher>, Topic> m_publishers;
};

}  // namespace ros
}  // namespace chrono

#endif
