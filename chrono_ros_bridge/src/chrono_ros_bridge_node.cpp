#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "chrono_ros_bridge/ChROSBridge.h"

#include "chrono_ros_bridge/converters/ChSystem_converter.h"
#include "chrono_ros_bridge/converters/ChVehicle_converter.h"
#include "chrono_ros_bridge/converters/ChCameraSensor_converter.h"
#include "chrono_ros_bridge/converters/ChDriverInputs_converter.h"

using namespace rapidjson;
using namespace chrono::ros;

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto bridge = std::make_shared<ChROSBridge>();
    rclcpp::spin(bridge);

    rclcpp::shutdown();
    return 0;
}

