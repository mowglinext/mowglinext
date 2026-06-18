cmake_minimum_required(VERSION 3.8)
project(mowgli_tools)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/tune_drive_pid
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(test_drive_pid_math
    test/test_drive_pid_math.py
    TIMEOUT 30
  )
endif()

ament_package()
