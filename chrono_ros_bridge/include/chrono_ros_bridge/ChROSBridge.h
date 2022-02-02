#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "chrono_ros_bridge/ChSocket.h"

namespace chrono {
namespace ros {

class ChROSBridge : public rclcpp::Node {
  public:
    ChROSBridge();

  private:
    void TimerCallback();

    rclcpp::TimerBase::SharedPtr m_timer;

    std::unique_ptr<chrono::utils::ChSocketTCP> m_client;
};

}  // namespace ros
}  // namespace chrono
