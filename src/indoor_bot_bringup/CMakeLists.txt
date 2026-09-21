cmake_minimum_required(VERSION 3.8)
project(indoor_bot_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Copyright linter disabled: licence is declared repo-wide in LICENSE and
  # in package.xml, per-file headers are not used in this project.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
