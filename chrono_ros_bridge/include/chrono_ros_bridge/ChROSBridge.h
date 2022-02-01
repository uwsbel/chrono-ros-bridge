#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "chrono/socket/ChSocket.h"
#include "chrono/socket/ChExceptionSocket.h"

namespace chrono {
namespace ros {

class ChROSBridge : public rclcpp::Node {
  public:
    ChROSBridge();

  private:
    void TimerCallback();

    rclcpp::TimerBase::SharedPtr m_timer;

    std::unique_ptr<chrono::socket::ChSocketTCP> m_client;
};

}  // namespace ros
}  // namespace chrono
