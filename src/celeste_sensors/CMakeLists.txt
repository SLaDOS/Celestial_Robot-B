cmake_minimum_required(VERSION 3.8)
project(celeste_sensors)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include)


## Declare a C++ library
add_library(
  ADS122C04
  lib/ADS122C04_ADC_PI.cpp
  )

add_library(
  POL_OP
  lib/POL_OP.cpp
  )

target_link_libraries(POL_OP
  ADS122C04
)

add_executable(pol_op_mux src/pol_op_mux.cpp)
ament_target_dependencies(pol_op_mux rclcpp std_msgs)

target_link_libraries(pol_op_mux
  POL_OP
  stdc++fs
  i2c
)

install(TARGETS
  pol_op_mux
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
