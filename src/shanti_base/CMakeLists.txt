cmake_minimum_required(VERSION 3.8)
project(shanti_base)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Uncomment if you add executables later
# add_executable(your_node src/your_node.cpp)
# ament_target_dependencies(your_node
#   rclcpp
#   geometry_msgs
#   sensor_msgs
# )

# Uncomment if you add libraries later
# add_library(${PROJECT_NAME}
#   src/some_file.cpp
# )
# ament_target_dependencies(${PROJECT_NAME}
#   rclcpp
#   geometry_msgs
#   sensor_msgs
# )

# Install launch files
install(DIRECTORY
  launch
  src
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# Uncomment to install executables when you add them
# install(TARGETS
#   your_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# Uncomment if you have header files to install
# install(
#   DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION include/${PROJECT_NAME}
# )

# Export dependencies
ament_export_dependencies(
  rclcpp
  geometry_msgs
  sensor_msgs
)

# Uncomment if you create libraries
# ament_export_include_directories(include)
# ament_export_libraries(${PROJECT_NAME})

ament_package()