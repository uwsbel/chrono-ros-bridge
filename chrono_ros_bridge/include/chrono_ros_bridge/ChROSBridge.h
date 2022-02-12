#ifndef CH_ROS_BRIDGE_H
#define CH_ROS_BRIDGE_H

#include <memory>
#include <functional>
#include <chrono>
#include <tuple>
#include <map>

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/reader.h>

#include "chrono_ros_bridge/ChSocket.h"
#include "chrono_ros_bridge/converters/common.h"

namespace chrono {
namespace ros {

class ChROSBridge : public rclcpp::Node {
  public:
    ChROSBridge();

    /**
     * Publish helper method.
     *
     * Will serialize a generic message and publish using the generic publisher.
     */
    template <typename MessageT>
    void Publish(const std::string& name, const std::string& type, const MessageT& msg, const rclcpp::QoS& qos = 1) {
        auto& publisher = m_publishers[type];
        if (not publisher)
            publisher = create_generic_publisher(name, rosidl_generator_traits::name<MessageT>(), qos);

        static rclcpp::Serialization<MessageT> serializer;
        static rclcpp::SerializedMessage serialized_msg;

        serializer.serialize_message(&msg, &serialized_msg);

        publisher->publish(serialized_msg);
    }

    /**
     * Add a message parser.
     *
     * A message parser is defined as a function that converts data _from_ json _to_ a ROS message.
     *
     * If the specified type has already been added to the map, it will overwrite the previous one.
     *
     * It must have the following signature:
     * std::function<void(const rapidjson::Value&, ChROSBridge*)>
     */
    void AddMessageParser(const std::string& type, MessageParserFunc parser_func);

    void CreateSubscription(ChROSMessageGenerator& gen);

  private:
    void TimerCallback();

    // -----
    // Other
    // -----

    rclcpp::TimerBase::SharedPtr m_timer;

    std::unique_ptr<chrono::utils::ChSocketTCP> m_client;

    std::map<std::string, std::shared_ptr<rclcpp::GenericSubscription>> m_subscribers;

    std::map<std::string, MessageParserFunc> m_message_parsers;
    std::map<std::string, std::shared_ptr<rclcpp::GenericPublisher>> m_publishers;
};

#define CH_REGISTER_MESSAGE_PARSER(type, parser)                    \
    namespace parser_registry {                                     \
    static ChROSMessageParserStorage parser##_parser(type, parser); \
    }

#define CREATE_CALLBACK(generator, type, topic_name)

#define CH_REGISTER_MESSAGE_GENERATOR(type, topic_name, topic_type, generator)                                       \
    namespace parser_registry {                                                                                      \
    static auto generator##_callback = [](std::shared_ptr<rclcpp::SerializedMessage> msg) {                          \
        auto& writer = ChROSBridgeWriter::GetInstance();                                                             \
                                                                                                                     \
        std::scoped_lock lock(writer.mutex);                                                                         \
                                                                                                                     \
        writer.StartObject();                                                                                        \
                                                                                                                     \
        {                                                                                                            \
            writer.Key("type");                                                                                      \
            writer.String(type);                                                                                     \
        }                                                                                                            \
                                                                                                                     \
        {                                                                                                            \
            writer.Key("name");                                                                                      \
            writer.String(topic_name);                                                                               \
        }                                                                                                            \
                                                                                                                     \
        {                                                                                                            \
            writer.Key("data");                                                                                      \
            writer.StartObject();                                                                                    \
            generator(msg, writer);                                                                                  \
                                                                                                                     \
            writer.EndObject();                                                                                      \
        }                                                                                                            \
        writer.EndObject();                                                                                          \
    };                                                                                                               \
    static ChROSMessageGeneratorStorage generator##_generator({type, topic_name, topic_type, generator##_callback}); \
    }

}  // namespace ros
}  // namespace chrono

#endif
