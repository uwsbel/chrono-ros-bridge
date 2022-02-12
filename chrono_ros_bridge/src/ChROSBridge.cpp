#include "chrono_ros_bridge/ChROSBridge.h"

using namespace rapidjson;
using namespace chrono::utils;
using namespace std::chrono_literals;

namespace chrono {
namespace ros {

ChROSBridge::ChROSBridge() : Node("chrono_ros_bridge") {
    using namespace std::placeholders;

    // Declare parameters
    auto port = this->declare_parameter("port", 50000);
    auto ip = this->declare_parameter("ip", "127.0.0.1");

    // Create the client
    m_client = std::make_unique<ChSocketTCP>(port);
    m_client->connectToServer(ip, utils::ADDRESS);

    // Create all the subscribers
    for (auto& generator : ChROSMessageGeneratorStorage::message_generators)
        CreateSubscription(generator);

    // Register parsers
    MessageParserFunc parser;
    for (auto& [type, parser] : ChROSMessageParserStorage::message_parsers)
        AddMessageParser(type, parser);

    m_timer = this->create_wall_timer(1us, std::bind(&ChROSBridge::TimerCallback, this));

    ChROSBridgeWriter::GetInstance().StartArray();
}

void ChROSBridge::CreateSubscription(ChROSMessageGenerator& gen) {
    m_subscribers[gen.type] = this->create_generic_subscription(gen.topic_name, gen.topic_type, 1, gen.generator);
}
void ChROSBridge::AddMessageParser(const std::string& type,
                                   std::function<void(const rapidjson::Value&, ChROSBridge*)> parser_func) {
    if (m_message_parsers.count(type))
        RCLCPP_WARN_STREAM(rclcpp::get_logger("ChSocket"), type << " is already in the map. Replacing.");
    m_message_parsers[type] = parser_func;
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
                    m_message_parsers[type](m, this);
            }
        }

        // ----
        // Send
        // ----
        {
            auto& writer = ChROSBridgeWriter::GetInstance();

            std::scoped_lock lock(writer.mutex);

            writer.EndArray();
            std::string message(writer.buffer.GetString());
            m_client->sendMessage(message);

            // Restart the buffer
            writer.buffer.Clear();
            writer.Reset(writer.buffer);
            writer.StartArray();
        }
    } catch (utils::ChExceptionSocket& exception) {
        std::cout << " ERRROR with socket system: \n" << exception.what() << std::endl;
    }
}

}  // namespace ros
}  // namespace chrono
