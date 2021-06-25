cmake_minimum_required(VERSION 3.5)
project(kmr_navigation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(files/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  PROGRAMS
      files/keyboard1.py
      files/keyboard2.py
      files/navigate_point_scan1.py
      files/navigate_point_scan2.py
      files/navigate_waypoint_scan1.py
      files/navigate_waypoint_scan2.py
      files/trajectory_kuka1.py
      files/trajectory_kuka2.py
      files/CA_kuka1.py
      files/CA_kuka1_EFH.py
      files/CA_kuka2.py
      files/CAET_kuka1.py
      files/CAET_kuka1_EFH.py
      files/CAET_kuka2.py
      files/CAE_kuka1.py
      files/CAE_kuka1_EFH.py
      files/CAE_kuka2.py

  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
