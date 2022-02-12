#ifndef COMMON_H
#define COMMON_H

#include <memory>
#include <functional>

#include <rclcpp/serialization.hpp>

#include "chrono_ros_bridge/ChJSONHelpers.h"

namespace chrono {
namespace ros {

typedef std::shared_ptr<rclcpp::SerializedMessage> SerializedMessagePtr;
typedef std::function<void(SerializedMessagePtr, chrono::utils::ChJSONWriter&)> SerializedMessageCallback;
typedef std::function<SerializedMessagePtr(chrono::utils::ChJSONReader&)> Json2RosConverter;

template <typename MessageT>
MessageT Deserialize(const SerializedMessagePtr msg) {
    static rclcpp::Serialization<MessageT> serializer;

    MessageT deserialized_msg;
    serializer.deserialize_message(msg.get(), &deserialized_msg);
    return deserialized_msg;
}

template <typename MessageT>
SerializedMessagePtr Serialize(const MessageT& msg) {
    static rclcpp::Serialization<MessageT> serializer;

    auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
    serializer.serialize_message(&msg, serialized_msg.get());
    return serialized_msg;
}

}  // namespace ros
}  // namespace chrono

#endif
