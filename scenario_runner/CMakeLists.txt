cmake_minimum_required(VERSION 3.5)
project(scenario_runner)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic)
endif()

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
  ${YAML_CPP_INCLUDE_DIR}
)

ament_auto_add_library(scenario_runner SHARED
  src/scenario_terminator.cpp
  src/scenario_runner.cpp)
target_link_libraries(scenario_runner
  glog
)

ament_auto_add_executable(scenario_runner_node
  src/scenario_runner_node.cpp)
target_link_libraries(scenario_runner_node
  scenario_runner
  ${YAML_CPP_LIBRARIES}
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
