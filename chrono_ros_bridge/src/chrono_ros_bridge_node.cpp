#include "chrono_ros_bridge/ChROSBridge.h"

#include "chrono_ros_bridge/converters/ChCameraSensor_converter.h"
#include "chrono_ros_bridge/converters/ChSystem_converter.h"
#include "chrono_ros_bridge/converters/ChVehicle_converter.h"
#include "chrono_ros_bridge/converters/ChIMUSensor_converter.h"
#include "chrono_ros_bridge/converters/ChGPSSensor_converter.h"

#include "chrono_ros_bridge/converters/ChDriverInputs_converter.h"

using namespace chrono::ros;

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto bridge = std::make_shared<chrono::ros::ChROSBridge>();

    bridge->AddMessageParser<rosgraph_msgs::msg::Clock>("ChSystem", ChSystem_converter);
    bridge->AddMessageParser<chrono_ros_msgs::msg::ChVehicle>("ChVehicle", ChVehicle_converter);
    bridge->AddMessageParser<sensor_msgs::msg::Image>("ChCameraSensor", ChCameraSensor_converter);
    bridge->AddMessageParser<sensor_msgs::msg::Imu>("ChAccelerometerSensor", ChAccelerometerSensor_converter);
    bridge->AddMessageParser<sensor_msgs::msg::Imu>("ChGyroscopeSensor", ChGyroscopeSensor_converter); 
    bridge->AddMessageParser<sensor_msgs::msg::MagneticField>("ChMagnetometerSensor", ChMagnetometerSensor_converter); 
    bridge->AddMessageParser<sensor_msgs::msg::NavSatFix>("ChGPSSensor", ChGPSSensor_converter); 

    bridge->AddMessageGenerator<chrono_ros_msgs::msg::ChDriverInputs>({"~/input/driver_inputs", "ChDriverInputs"},
                                                                      ChDriverInputs_converter);

    rclcpp::spin(bridge);
    rclcpp::shutdown();
    return 0;
}

