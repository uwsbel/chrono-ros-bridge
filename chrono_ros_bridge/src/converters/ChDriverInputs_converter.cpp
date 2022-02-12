#include "chrono_ros_bridge/converters/ChDriverInputs_converter.h"

#include "chrono_ros_bridge/ChROSBridge.h"

using namespace rapidjson;

namespace chrono {
namespace ros {

void ChDriverInputs_converter(std::shared_ptr<rclcpp::SerializedMessage>& serialized_msg,
                              Writer<StringBuffer>& writer) {
    auto msg = Deserialize<chrono_ros_msgs::msg::ChDriverInputs>(serialized_msg);

    writer.Key("steering");
    writer.Double(msg.steering);
    writer.Key("throttle");
    writer.Double(msg.throttle);
    writer.Key("braking");
    writer.Double(msg.braking);
}

CH_REGISTER_MESSAGE_GENERATOR("ChDriverInputs",
                              "~/input/ChDriverInputs",
                              rosidl_generator_traits::name<chrono_ros_msgs::msg::ChDriverInputs>(),
                              ChDriverInputs_converter)

}  // namespace ros
}  // namespace chrono
