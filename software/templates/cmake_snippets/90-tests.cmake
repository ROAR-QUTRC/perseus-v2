# TESTING
if(NOT DEFINED BUILD_TESTING)
  option(BUILD_TESTING "Build tests" ON)
endif()
if(BUILD_TESTING)
  enable_testing()
  find_package(GTest REQUIRED)

  add_executable(packet_manager_tests tests/packet_manager.cpp)
  target_link_libraries(packet_manager_tests hi_can GTest::gtest_main)
  target_include_directories(packet_manager_tests
                             PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

  add_executable(standard_address_tests tests/standard_address.cpp)
  target_link_libraries(standard_address_tests hi_can GTest::gtest_main)
  target_include_directories(standard_address_tests
                             PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

  include(GoogleTest)
  gtest_discover_tests(packet_manager_tests)
  gtest_discover_tests(standard_address_tests)
endif()
