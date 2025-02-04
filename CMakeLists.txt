cmake_minimum_required(VERSION 3.5)
project(lidro_isl100)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
  include
)
link_directories(
  ${PCL_LIBRARY_DIRS}
  ${GLOG_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

# compile the driver and input library
add_subdirectory(src/lib)

# build the driver node
add_library(${PROJECT_NAME} SHARED 
  src/driver.cpp
  src/interface.cpp
  src/frame.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  diagnostic_msgs
  diagnostic_updater
  rclcpp
  rclcpp_components
  tf2_ros
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2_eigen
  tf2_geometry_msgs
  tf2_sensor_msgs
  pcl_conversions
  cv_bridge
)
# ament_export_dependencies(Boost)

target_link_libraries(
  ${PROJECT_NAME}
  lidro_input
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


add_executable(${PROJECT_NAME}_node src/node_main.cpp)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

# install runtime and library files
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

rclcpp_components_register_nodes(${PROJECT_NAME}
  "lidro::isl100::Isl100Driver")

install(DIRECTORY include/lidro/isl100
  DESTINATION include/lidro/isl100)

install(DIRECTORY config launch rviz urdf
  DESTINATION share/${PROJECT_NAME})

ament_package()
