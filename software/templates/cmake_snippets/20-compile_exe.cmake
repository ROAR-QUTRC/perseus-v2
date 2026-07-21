# COMPILE (Executables)
foreach(node IN ITEMS ${NODE_NAMES})
  # Find all source files for the executables
  file(GLOB_RECURSE CODE_SOURCES src/${node}/*.cpp)

  # Add exec targets
  add_executable(${node} ${CODE_SOURCES})

  # Add include dirs
  target_include_directories(${node} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
endforeach()
