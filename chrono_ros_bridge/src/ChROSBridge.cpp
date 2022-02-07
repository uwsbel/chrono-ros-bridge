#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstring>
#include <codecvt>  // codecvt_utf8
#include <locale>   // wstring_convert

#include "chrono_ros_bridge/ChROSBridge.h"

#include "chrono_ros_bridge/thirdparty/nlohmann/json.hpp"
#include "chrono_ros_bridge/thirdparty/utf8.h"

using namespace std::chrono_literals;
using namespace chrono::utils;
using json = nlohmann::json;

// encoding function
void fix_utf8_string(std::string& str) {
    std::string temp;
    utf8::replace_invalid(str.begin(), str.end(), back_inserter(temp));
    str = temp;
}

namespace chrono {
namespace ros {

ChROSBridge::ChROSBridge() : Node("chrono_ros_bridge") {
    // TODO: Make a parameter to get port
    m_client = std::make_unique<ChSocketTCP>(50000);
    m_client->connectToServer("128.104.190.1", utils::ADDRESS);

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
            message += '\0';

            // Parse the JSON string
            try {
                fix_utf8_string(message);
                auto j = json::parse(message);

                for (auto& m : j) {
                    std::string type = m["type"].get<std::string>();
                    if (type == "ChCameraSensor") {
                        auto data = m["data"];

                        if (data.contains("image")) {
                            uint64_t total_len = data["width"].get<uint64_t>() * data["height"].get<uint64_t>() *
                                                 data["size"].get<uint64_t>();

                            std::string image_msg = data["image"].get<std::string>();
                            std::vector<uint8_t> image(image_msg.c_str(), image_msg.c_str() + total_len);
                        }
                    }
                }
            } catch (json::parse_error& e) {
                // output exception information
                std::cout << "message: " << e.what() << '\n'
                          << "exception id: " << e.id << '\n'
                          << "byte position of error: " << e.byte << std::endl;
                std::cout << message.at(e.byte) << std::endl;
                std::cout << message.substr(e.byte - 10, e.byte) << std::endl;
                throw e;
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

}  // namespace ros
}  // namespace chrono

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<chrono::ros::ChROSBridge>());
    rclcpp::shutdown();
    return 0;
}
