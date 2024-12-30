cmake_minimum_required(VERSION 3.5)
project(auto_localization)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Find SQLite3
find_package(PkgConfig REQUIRED)
pkg_check_modules(SQLITE3 REQUIRED sqlite3)

# Include directories for SQLite3 and other dependencies
include_directories(
  ${SQLITE3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Add the executable for the node
add_executable(auto_localization_node src/AutoLocalization.cpp)

# Link SQLite3 library
target_link_libraries(auto_localization_node ${SQLITE3_LIBRARIES})

# Link ROS 2 dependencies
ament_target_dependencies(auto_localization_node rclcpp geometry_msgs tf2 tf2_ros)

# Ensure the data folder is installed along with the package
install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)


# Install the include folder
install(DIRECTORY include/
  DESTINATION include/
)

# Install the executable
install(TARGETS auto_localization_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
