cmake_minimum_required(VERSION 3.8)
project(client_within_action_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wunused)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_tutorials_interfaces REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(client_within_action_1 src/client_within_action_1.cpp)
ament_target_dependencies(client_within_action_1
  rclcpp
  rclcpp_action
  action_tutorials_interfaces
  std_msgs
)

add_executable(client_within_action_2 src/client_within_action_2.cpp)
ament_target_dependencies(client_within_action_2
  rclcpp
  rclcpp_action
  action_tutorials_interfaces
  std_msgs
)

add_executable(client_within_action_3 src/client_within_action_3.cpp)
ament_target_dependencies(client_within_action_3
  rclcpp
  rclcpp_action
  action_tutorials_interfaces
  std_msgs
)

add_executable(client_within_action_4 src/client_within_action_4.cpp)
ament_target_dependencies(client_within_action_4
  rclcpp
  rclcpp_action
  action_tutorials_interfaces
  std_msgs
)

add_executable(client_within_action_5 src/client_within_action_5.cpp)
ament_target_dependencies(client_within_action_5
  rclcpp
  rclcpp_action
  action_tutorials_interfaces
  std_msgs
)

add_executable(client_within_action_6 src/client_within_action_6.cpp)
ament_target_dependencies(client_within_action_6
  rclcpp
  rclcpp_action
  action_tutorials_interfaces
  std_msgs
)

add_executable(client_within_action_7 src/client_within_action_7.cpp)
ament_target_dependencies(client_within_action_7
  rclcpp
  rclcpp_action
  action_tutorials_interfaces
  std_msgs
)

add_executable(client_within_action_8 src/client_within_action_8.cpp)
ament_target_dependencies(client_within_action_8
  rclcpp
  rclcpp_action
  action_tutorials_interfaces
  std_msgs
)

add_executable(client_within_action_9 src/client_within_action_9.cpp)
ament_target_dependencies(client_within_action_9
  rclcpp
  rclcpp_action
  action_tutorials_interfaces
  std_msgs
)

install(TARGETS
  client_within_action_1
  client_within_action_2
  client_within_action_3
  client_within_action_4
  client_within_action_5
  client_within_action_6
  client_within_action_7
  client_within_action_8
  client_within_action_9
  
  DESTINATION lib/${PROJECT_NAME})

ament_package()