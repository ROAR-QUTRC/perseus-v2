cmake_minimum_required(VERSION 3.8)
project(hardware_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# drive node
add_executable(drive src/drive.cpp)
target_include_directories(drive PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(drive PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS drive
  DESTINATION lib/${PROJECT_NAME})

# beacon node
add_executable(beacon src/beacon.cpp)
target_include_directories(beacon PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(beacon PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS beacon
  DESTINATION lib/${PROJECT_NAME})

# excavation node
add_executable(excavation src/excavation.cpp)
target_include_directories(excavation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(excavation PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS excavation
  DESTINATION lib/${PROJECT_NAME})

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

# check if hi_can_lib is already present (Nix build), if not, add its subdirectory
find_package(hi_can_lib QUIET)
if(NOT hi_can_lib_FOUND)
    message(STATUS "hi_can_lib not found, adding subdirectory...")
    # add the directory with its CMakeLists.txt
    add_subdirectory(../../shared/hi-can-lib)
endif()

target_link_libraries(drive PRIVATE hi_can_lib)
target_link_libraries(beacon PRIVATE hi_can_lib)
