/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2022 University of Wisconsin - Madison
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
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
