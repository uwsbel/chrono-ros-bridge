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

