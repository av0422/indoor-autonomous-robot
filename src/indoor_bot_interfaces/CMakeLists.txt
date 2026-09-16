cmake_minimum_required(VERSION 3.8)
project(indoor_bot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# msg/ and action/ are intentionally empty placeholders — no interfaces
# defined yet. rosidl_generate_interfaces() with no files still produces
# a valid, empty generated package.
rosidl_generate_interfaces(${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
