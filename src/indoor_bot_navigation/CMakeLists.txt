cmake_minimum_required(VERSION 3.8)
project(indoor_bot_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  maps
  launch
  foxglove_layouts
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
