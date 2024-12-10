# INSTALL (ROS outputs)
install(DIRECTORY description/launch description/urdf
        DESTINATION share/${PROJECT_NAME})
install(DIRECTORY bringup/launch bringup/config
        DESTINATION share/${PROJECT_NAME})
