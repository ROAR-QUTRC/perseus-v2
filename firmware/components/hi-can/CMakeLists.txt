# Register the component
idf_component_register(
  SRC_DIRS
  ../../../software/shared/hi-can/src
  ./src
  INCLUDE_DIRS
  ../../../software/shared/hi-can/include
  ./include
  REQUIRES
  board-support
  driver)
# # set(BUILD_SHARED_LIBS OFF) #
# add_subdirectory(../../../../software/shared/hi-can hi-can) #
# target_link_libraries(${COMPONENT_LIB} INTERFACE hi-can) # To build a
# third-party library, ExternalProject CMake module can be used. #
# ExternalProject offers many features which are impossible to demonstrate # in
# a single example. Please refer to its documentation for more info: #
# https://cmake.org/cmake/help/latest/module/ExternalProject.html
# include(ExternalProject)

# # Define the location where tinyxml2 will be installed: set(HI_CAN_INSTALL_DIR
# ${CMAKE_CURRENT_BINARY_DIR}/hi_can_install)

# # This function downloads the project, calls CMake to configure it, # builds
# the project and installs it to the specified location:
# externalproject_add(hi_can_proj URL ../../../../software/shared/hi-can/
# DOWNLOAD_NO_EXTRACT TRUE CMAKE_ARGS # Use the same CMake toolchain file as for
# the main project. -DBUILD_SHARED_LIBS=OFF

# # These options are set so that Ninja immediately outputs # the subproject
# build to the terminal. Otherwise it looks like the # build process "hangs"
# while the subproject is being built. USES_TERMINAL_CONFIGURE TRUE
# USES_TERMINAL_BUILD TRUE

# # Specify the installation directory for the subproject INSTALL_DIR
# ${HI_CAN_INSTALL_DIR} # Let CMake know that the library is generated by the
# subproject build step. BUILD_BYPRODUCTS "${HI_CAN_INSTALL_DIR}/lib/hi-can.a" )

# # Now that the subproject build is set up, we need to consume the results # of
# the build: the header file and the static library. # To do this, define an
# imported CMake library: add_prebuilt_library(hi_can_lib
# "${HI_CAN_INSTALL_DIR}/lib/hi-can.a" # tinyxml calls certain C++ support
# library functions (_Unwind_Resume and similar) # so a dependency on IDF's cxx
# component is added here: PRIV_REQUIRES cxx)
# target_include_directories(hi_can_lib INTERFACE
# "${HI_CAN_INSTALL_DIR}/hi-can/include") add_dependencies(hi_can_lib
# hi_can_proj)

# # Link the imported library to the current component.
# target_link_libraries(${COMPONENT_LIB} INTERFACE hi_can_lib)
