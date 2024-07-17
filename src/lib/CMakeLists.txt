add_library(lidro_input SHARED
  udp.cpp
)
ament_target_dependencies(lidro_input
  rclcpp
)
target_link_libraries(lidro_input
  ${libpcap_LIBRARIES}
)

install(TARGETS lidro_input
  LIBRARY DESTINATION lib
)
