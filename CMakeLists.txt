cmake_minimum_required(VERSION 3.0.2)
project(localization_pkg)

## Compile as C++11
add_compile_options(-std=c++11)

## Find catkin and required components
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  humanoid_nav_msgs
  std_srvs
  message_generation
  tf
)

add_service_files(
  FILES
  GetRelativeFootsteps.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  humanoid_nav_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs humanoid_nav_msgs std_srvs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare the executable
add_executable(soccer_localization_node
  src/soccer_localization_node.cpp
)

## Link the executable to catkin libraries
target_link_libraries(soccer_localization_node
  ${catkin_LIBRARIES}
)

## Add dependencies to ensure proper build order
add_dependencies(soccer_localization_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
