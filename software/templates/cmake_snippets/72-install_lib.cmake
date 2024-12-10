# INSTALL (Libraries)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# CMake packaging - see https://blog.vito.nyc/posts/cmake-pkg/ for a good
# explanation
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.cmake.in
               ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake @ONLY)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
  COMPATIBILITY SameMinorVersion)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
        DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME})

# Headers
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

# Versioning
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${PROJECT_VERSION})
# The main install - default locations are fine
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_targets
  ARCHIVE
  LIBRARY
  RUNTIME)

# The last of the CMake packaging info
install(EXPORT ${PROJECT_NAME}_targets
        DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})
