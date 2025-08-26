# This file was automatically generated for projects without default
# 'CMakeLists.txt' file.

file(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(
  SRCS
  ${app_sources}
  REQUIRES
  board-support
  crc
  hi-can
  type
  PRIV_REQUIRES)

# Create and import the library targets
# add_subdirectory(../../../software/shared/hi-can hi-can)

# Publicly link `foo` to `main` component target_link_libraries(main PUBLIC
# hi-can)
