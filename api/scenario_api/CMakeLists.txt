cmake_minimum_required(VERSION 3.5)
project(scenario_api)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # TODO remove -Werror before merging
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED COMPONENTS common )

include_directories(${PCL_INCLUDE_DIRS})


ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/scenario_api_calc_dist_utils.cpp
  src/scenario_api_coordinate_manager.cpp
  src/scenario_api_core.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
)

ament_export_dependencies(PCL)

ament_auto_add_executable(${PROJECT_NAME}_node
  src/scenario_api_node.cpp
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
  