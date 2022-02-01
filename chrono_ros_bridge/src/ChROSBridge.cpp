#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "chrono_ros_bridge/ChROSBridge.h"

#include "chrono/core/ChStream.h"

using namespace std::chrono_literals;

using namespace chrono::socket;

namespace chrono {
namespace ros {

ChROSBridge::ChROSBridge() : Node("chrono_ros_bridge") {
    // TODO: Make a parameter to get port
    m_client = std::make_unique<ChSocketTCP>(50000);
    m_client->connectToServer("host.docker.internal", socket::NAME);

    m_timer = this->create_wall_timer(1us, std::bind(&ChROSBridge::TimerCallback, this));
}

void ChROSBridge::TimerCallback() {
    try {
        // ----
        // Send
        // ----
        std::vector<char> buffer_out;
        ChStreamOutBinaryVector stream_out(&buffer_out);  // wrap the buffer, for easy formatting

        double throttle = 1.0;
        double steering = 0.0;
        double braking = 0.0;
        stream_out << throttle << steering << braking;

        m_client->SendBuffer(*stream_out.GetVector());

        // -------
        // Receive
        // -------

        int nbytes = sizeof(double);
        std::vector<char> rbuffer;
        rbuffer.resize(nbytes);                      // reserve to number of expected bytes
        ChStreamInBinaryVector stream_in(&rbuffer);  // wrap the buffer, for easy formatting

        m_client->ReceiveBuffer(*stream_in.GetVector(), nbytes);
    } catch (socket::ChExceptionSocket& exception) {
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
