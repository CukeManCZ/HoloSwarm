cmake_minimum_required(VERSION 3.8)
project(holo_swarm)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set dependencies
set(DEPENDENCIES
  ament_cmake
  rclcpp
  rclcpp_components
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  nav_msgs  
  std_srvs
  mrs_msgs
  mrs_lib
)

# find dependencies
foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED) 
endforeach(DEP IN LISTS DEPENDENCIES)

# libs
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${colcon_INCLUDE_DIRS}
  ${mrs_lib_INCLUDE_DIRS}
)

# MY SCRIPTS SETTING ----------------
add_library(single_uav SHARED src/single_uav.cpp)

target_include_directories(single_uav PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(single_uav
  ${DEPENDENCIES}
)

rclcpp_components_register_nodes(
  single_uav
  PLUGIN "single_uav::SingleUAV"
  EXECUTABLE single_uav
)

# INSTALLATION --------------
ament_export_targets(export_single_uav)
install(TARGETS single_uav
        EXPORT export_single_uav
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  include
  DESTINATION include
)

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
