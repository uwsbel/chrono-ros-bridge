cmake_minimum_required(VERSION 3.8)
project(chrono_ros_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)

include_directories(include)

add_executable(chrono_ros_bridge
	src/ChROSBridge.cpp
	src/chrono/ChStream.cpp
	src/chrono/ChSocket.cpp
	src/chrono/ChHostInfo.cpp
	src/chrono/ChSocketFramework.cpp)
ament_target_dependencies(chrono_ros_bridge rclcpp)

install(TARGETS
	chrono_ros_bridge	
	DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()