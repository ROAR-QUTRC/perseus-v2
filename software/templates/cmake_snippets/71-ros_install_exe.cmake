# INSTALL(ROS Executables)
foreach(node IN ITEMS ${NODE_NAMES})
  # Install ROS Executable
  install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
endforeach()
