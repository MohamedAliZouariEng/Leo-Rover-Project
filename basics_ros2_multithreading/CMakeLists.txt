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
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Get home directory and set the specific Python path
if(NOT DEFINED ENV{HOME})
  set(HOME_DIR "/home/tempuser")
else()
  set(HOME_DIR "$ENV{HOME}")
endif()

set(SPECIFIC_PYTHON_PATH "${HOME_DIR}/ros2_venv/bin/python3")
message(STATUS "Setting Python shebang to: ${SPECIFIC_PYTHON_PATH}")

# We add this to be able to import scripts form other scripts of our package
# Also to execute launch files in the launch folder
install(DIRECTORY
  scripts
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Define the list of Python scripts
set(PYTHON_SCRIPTS
  function.py
  callback_function.py
  callback_spinonce_function.py
  plant_detector.py
  plant_detector_multithreading.py
  plant_detector_multithreading_callbackgroups.py
  green_detector_node.py
  green_test.py
)

# Process each Python script
foreach(SCRIPT ${PYTHON_SCRIPTS})
  # Set input and output paths
  set(INPUT_FILE "${CMAKE_CURRENT_SOURCE_DIR}/scripts/${SCRIPT}")
  set(OUTPUT_FILE "${CMAKE_CURRENT_BINARY_DIR}/${SCRIPT}")
  
  # Read the original script
  file(READ ${INPUT_FILE} FILE_CONTENT)
  
  # Replace ANY shebang line with our specific one
  # This handles: #!/usr/bin/env python3, #!/usr/bin/python3, #!/usr/bin/python, etc.
  string(REGEX REPLACE "^#![^\n]*" "#!${SPECIFIC_PYTHON_PATH}" MODIFIED_CONTENT "${FILE_CONTENT}")
  
  # If no shebang was found (unlikely but handle it), prepend it
  if("${MODIFIED_CONTENT}" STREQUAL "${FILE_CONTENT}")
    set(MODIFIED_CONTENT "#!${SPECIFIC_PYTHON_PATH}\n${FILE_CONTENT}")
  endif()
  
  # Write the modified file
  file(WRITE ${OUTPUT_FILE} "${MODIFIED_CONTENT}")
  
  # Make it executable
  file(COPY ${OUTPUT_FILE}
       DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
       FILE_PERMISSIONS
       OWNER_READ OWNER_WRITE OWNER_EXECUTE
       GROUP_READ GROUP_EXECUTE
       WORLD_READ WORLD_EXECUTE)
  
  # Install the processed file
  install(PROGRAMS
    ${OUTPUT_FILE}
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${SCRIPT}
  )
  
  message(STATUS "Processed ${SCRIPT} with shebang: #!${SPECIFIC_PYTHON_PATH}")
endforeach()

ament_package()