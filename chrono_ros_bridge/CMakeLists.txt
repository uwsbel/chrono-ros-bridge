cmake_minimum_required(VERSION 3.8)
project(chrono_ros_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_ros REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.

set(DEPENDENCIES
	rclcpp
	rclcpp_components
	std_msgs
	geometry_msgs
	sensor_msgs
	rosgraph_msgs
	chrono_ros_msgs
)

foreach (dependency IN LISTS DEPENDENCIES)
  find_package(${dependency} REQUIRED)
endforeach()

ament_auto_find_build_dependencies(REQUIRED ${DEPENDENCIES})

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/chrono_ros_bridge_node.cpp

	src/ChROSBridge.cpp
	src/ChSocket.cpp
	src/ChJSONHelpers.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE 
  include
)
rclcpp_components_register_nodes(${PROJECT_NAME} "chrono::ros::ChROSBridge")

ament_auto_add_executable(${PROJECT_NAME}_node src/chrono_ros_bridge_node.cpp)

# the node must go into the project specific lib directory or else
# the launch file will not find it

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}/
)

# the shared library goes into the global lib dir so it can
# be used as a composable node by other projects

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib
)

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

ament_auto_package()
