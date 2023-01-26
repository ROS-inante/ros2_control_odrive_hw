cmake_minimum_required(VERSION 3.8)
project(ros2_control_odrive_hw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ros2_odrive REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

include_directories(include )

# COMPILE
add_library(
  ${PROJECT_NAME}
  SHARED
  src/ros2_control_odrive_hw.cpp
  src/ros2_control_dummy_odrive_hw.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
  ${ros2_odrive_INCLUDE_DIRS}
)

target_link_libraries(
  ${PROJECT_NAME} ros2_odrive::libodrive
)

ament_target_dependencies(
  ${PROJECT_NAME}
  ros2_odrive
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
## EXPORTS
pluginlib_export_plugin_description_file(hardware_interface
  ros2_control_odrive_hw.xml
)

## INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)

    list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_odrive REQUIRED)



  ament_add_gmock(test_odrive_system test/test_load_ros2_control_odrive_hw.cpp)
  target_include_directories(test_odrive_system PRIVATE include)

  ament_target_dependencies(
  test_odrive_system
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# target_link_libraries(test_odrive_system)
# ament_target_dependencies(test_odrive_system
#   pluginlib
#   ros2_control_test_assets
# )


   ament_lint_auto_find_test_dependencies()
 endif()


ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ros2_odrive
)

ament_package()
