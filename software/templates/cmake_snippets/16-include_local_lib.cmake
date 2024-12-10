# INCLUDE POSSIBLY LOCAL DEPENDENCIES

# check if libraries are already present (Nix build), if not, add them
find_package(hi_can QUIET)
if(NOT hi_can_FOUND)
  message(STATUS "hi_can not found, adding subdirectory...")
  # add the directory with its CMakeLists.txt
  add_subdirectory(../../../shared/hi-can hi-can)
endif()
