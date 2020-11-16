cmake_minimum_required(VERSION 3.5)
project(action_plugins)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # TODO remove -Werror before merging
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/targets.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

ament_auto_package(INSTALL_TO_SHARE
)
