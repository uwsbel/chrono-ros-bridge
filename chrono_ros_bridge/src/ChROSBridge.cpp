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
    auto ip = this->declare_parameter("ip", "");
    auto hostname = this->declare_parameter("hostname", "");

    // Create the client
    m_client = std::make_unique<ChSocketTCP>(port);
    if (not ip.empty())
        m_client->connectToServer(ip, utils::ADDRESS);
    else if (not hostname.empty())
        m_client->connectToServer(hostname, utils::NAME);
    else
        throw ChException("Either 'ip' or 'hostname' must be passed.");

    m_timer = this->create_wall_timer(1us, std::bind(&ChROSBridge::TimerCallback, this));
}

void ChROSBridge::Publish(const Topic topic, SerializedMessagePtr msg, const rclcpp::QoS& qos) {
    auto& publisher = m_publishers[topic];
    if (not publisher)
        publisher = this->create_generic_publisher(topic.id, m_topic_types[topic.type], qos);

    publisher->publish(*msg);
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
            m_reader.Parse(message);

            std::string type, id;
            while (m_reader.HasMembers()) {
                m_reader.StartObject() >> type >> id >> m_reader.Back() >> m_reader.Back();

                if (m_message_parsers.count(type)) {
                    auto serialized_msg = m_message_parsers[type](m_reader);
                    Publish({id, type}, serialized_msg);
                }

                m_reader.EndObject();
            }
        }

        // ----
        // Send
        // ----
        {
            std::string message = m_writer.Finish();
            m_client->sendMessage(message);
        }
    } catch (utils::ChExceptionSocket& exception) {
        std::cout << " ERRROR with socket system: \n" << exception.what() << std::endl;
    }
}

}  // namespace ros
}  // namespace chrono
