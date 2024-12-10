# TESTING
find_package(GTest REQUIRED)
enable_testing()

add_executable(test_name tests/test_file.cpp)
target_link_libraries(test_name GTest::gtest_main)
target_include_directories(test_name PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

include(GoogleTest)
gtest_discover_tests(test_name)
