cmake_minimum_required(VERSION 3.5)
project(entity_plugins)

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
  ${CMAKE_CURRENT_SOURCE_DIR}
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  # src/bicycle_entity.cpp
  # src/ego_entity.cpp
  # src/motobike_entity.cpp
  # src/pedestrian_entity.cpp
  # src/vehicle_entity.cpp
  src/targets.cpp
  )

ament_auto_package()
