cmake_minimum_required(VERSION 3.8)
project(basics_ros2_multithreading)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# We add this to be able to import scripts form other scripts of our package
# Also to execute launch files in the launch folder
install(DIRECTORY
  scripts
  launch
  DESTINATION share/${PROJECT_NAME}
)


# Install the other scripts normally (unchanged)
install(PROGRAMS
  scripts/function.py
  scripts/callback_function.py
  scripts/callback_spinonce_function.py
  scripts/plant_detector.py
  scripts/plant_detector_multithreading.py
  scripts/plant_detector_multithreading_callbackgroups.py
  scripts/green_detector_node.py
  scripts/green_test.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()