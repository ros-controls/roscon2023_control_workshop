cmake_minimum_required(VERSION 3.8)
project(tiago_chaining)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/tiago_chaining
)

ament_package()