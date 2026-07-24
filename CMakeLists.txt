cmake_minimum_required(VERSION 3.8)
project(localization_pkg)

# Default to C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

##############################################################################
# Find packages
##############################################################################

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(humanoid_nav_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

##############################################################################
# Services and messages
##############################################################################
  

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Referee.msg"
  "srv/GetRelativeFootsteps.srv"
  DEPENDENCIES
    std_msgs
    humanoid_nav_msgs
)

##############################################################################
# Include directories
##############################################################################

include_directories(
  include
)

##############################################################################
# Executable
##############################################################################

#add_executable(soccer_localization_node
 # src/soccer_localization_node.cpp
#)

#ament_target_dependencies(soccer_localization_node
 # rclcpp
 # std_msgs
  #std_srvs
  #tf2
  #tf2_ros
  #humanoid_nav_msgs
#)

##############################################################################
# Link generated service interfaces
##############################################################################

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

#target_link_libraries(soccer_localization_node
 # ${cpp_typesupport_target}
#)

##############################################################################
# Install
#### find_package(<dependency> REQUIRED)###########################################################################

#install(TARGETS
 # soccer_localization_node
  #DESTINATION lib/${PROJECT_NAME}
#)

##############################################################################
# Export
##############################################################################

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

##############################################################################
# Export
##############################################################################
ament_export_dependencies(rosidl_default_runtime)
ament_package()