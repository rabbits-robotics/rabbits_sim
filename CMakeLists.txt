cmake_minimum_required(VERSION 3.8)
project(rabbits_sim)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rabcl REQUIRED)

include_directories(include)

add_library(rabbits_sim_lib STATIC
  src/pty_link.cpp
)
ament_target_dependencies(rabbits_sim_lib rabcl)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_pty_link test/test_pty_link.cpp)
  target_link_libraries(test_pty_link rabbits_sim_lib util)
endif()

ament_package()
