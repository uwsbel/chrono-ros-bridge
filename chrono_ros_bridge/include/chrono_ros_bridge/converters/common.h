#ifndef CONVERTERS_COMMON_H
#define CONVERTERS_COMMON_H

#include <map>
#include <vector>
#include <mutex>
#include <memory>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/reader.h>

namespace chrono {
namespace ros {

class ChROSBridge;

using SerializedMessagePtr = std::shared_ptr<rclcpp::SerializedMessage>;
using MessageGeneratorFunc = std::function<void(SerializedMessagePtr)>;

using MessageParserFunc = std::function<void(const rapidjson::Value& v, ChROSBridge*)>;

/**
 * Helper class for holding static writer objects for all other storage mechanisms
 */
struct ChROSBridgeWriter : public rapidjson::Writer<rapidjson::StringBuffer> {
    inline static std::mutex mutex;
    inline static rapidjson::StringBuffer buffer;

    static ChROSBridgeWriter& GetInstance() {
        static ChROSBridgeWriter instance(buffer);
        return instance;
    }

  private:
    ChROSBridgeWriter(rapidjson::StringBuffer& buffer) : rapidjson::Writer<rapidjson::StringBuffer>(buffer) {}
};

/**
 * Holds information about a message generator
 */
struct ChROSMessageGenerator {
    std::string type;
    std::string topic_name;
    std::string topic_type;
    MessageGeneratorFunc generator;
};

/**
 * Stores registered parsers
 */
struct ChROSMessageParserStorage {
    ChROSMessageParserStorage(std::string type, MessageParserFunc func) { message_parsers[type] = func; }
    inline static std::map<std::string, MessageParserFunc> message_parsers;
};

/**
 * Stores registered generators
 */
struct ChROSMessageGeneratorStorage {
    ChROSMessageGeneratorStorage(ChROSMessageGenerator generator) { message_generators.push_back(generator); }
    inline static std::vector<ChROSMessageGenerator> message_generators;
};

/**
 * Deserialize message
 */
template <typename MessageT>
MessageT Deserialize(SerializedMessagePtr serialized_msg) {
    static rclcpp::Serialization<MessageT> serializer;

    MessageT deserialized_msg;
    serializer.deserialize_message(serialized_msg.get(), &deserialized_msg);

    return deserialized_msg;
}

}  // namespace ros
}  // namespace chrono

#endif
