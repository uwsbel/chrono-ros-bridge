#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "chrono_ros_bridge/ChROSBridge.h"

using namespace std::chrono_literals;

using namespace chrono::utils;

namespace chrono {
namespace ros {

ChROSBridge::ChROSBridge() : Node("chrono_ros_bridge") {
    // TODO: Make a parameter to get port
    m_client = std::make_unique<ChSocketTCP>(50000);
    m_client->connectToServer("host.docker.internal", utils::NAME);

    m_timer = this->create_wall_timer(1us, std::bind(&ChROSBridge::TimerCallback, this));
}

void ChROSBridge::TimerCallback() {
    try {
        // ----
        // Send
        // ----
        {
            std::string message =
                "[{\"type\":\"ChDriverInputs\",\"data\":{\"throttle\":1.0,\"steering\":0.0,\"braking\":0.0}}]";
            m_client->sendMessage(message);
        }

        // -------
        // Receive
        // -------
        {
            std::string message;
            m_client->receiveMessage(message);
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
