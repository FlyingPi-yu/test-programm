
cmake_minimum_required(VERSION 3.5)
project(rt_regler_simu_cpp)

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
find_package(std_msgs REQUIRED)
find_package(rttest REQUIRED)

add_executable(IMU src/eingangssignal_funktion.cpp)
ament_target_dependencies(IMU rclcpp std_msgs rttest)

add_executable(Regler src/bearbeitete_signal_funktion.cpp)
ament_target_dependencies(Regler rclcpp std_msgs rttest)

add_executable(Logger src/logger.cpp)
ament_target_dependencies(Logger rclcpp std_msgs rttest)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


install(TARGETS
  IMU
  Regler
  Logger
  DESTINATION lib/${PROJECT_NAME})

ament_package()
