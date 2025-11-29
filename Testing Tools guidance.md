Markdown

```
# ROS 2 Unit & Integration Testing Best Practices
For Large Robotics Projects with Hardware (e.g. Perseus-v2)

> Goal: Catch regressions early, enable safe refactoring, and run the full test suite in CI **without real hardware**.

## 1. Core Principles

- Separate **pure logic** from **ROS 2 communication** and **hardware access**
- Unit-test everything that is pure C++/Python (algorithms, state machines, filters, planners, etc.)
- Mock hardware interfaces and ROS dependencies in unit tests
- Use integration/launch tests for node-to-node and node-hardware interaction
- Never require real hardware in regular CI

## 2. Recommended Package Layout
```

my_package/ ├── CMakeLists.txt ├── package.xml ├── include/my_package/          # C++ headers ├── src/                         # Implementation ├── test/ │   ├── gtest/                   # C++ GoogleTest files │   ├── pytest/                  # Python pytest files │   ├── mocks/                   # Mock classes / fixtures │   └── launch/                  # Integration launch tests └── resource/my_package          # colcon marker file

text

```
## 3. CMakeLists.txt Template (ament_cmake)

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_package LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
# ... add your other dependencies ...

add_library(${PROJECT_NAME} SHARED src/my_node.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp rclcpp_components)
target_include_directories(${PROJECT_NAME} PUBLIC
  $$ <BUILD_INTERFACE: $${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# ==================== TESTING ====================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # GoogleTest – unit tests
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_gtest
    test/gtest/test_pure_logic.cpp
    test/gtest/test_node_with_mocks.cpp
    TIMEOUT 60
  )
  target_link_libraries(${PROJECT_NAME}_gtest ${PROJECT_NAME})
  ament_target_dependencies(${PROJECT_NAME}_gtest rclcpp)

  # pytest – Python tests
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(python_tests test/pytest TIMEOUT 120)

  # Launch / integration tests
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/launch/integration_test.launch.py TIMEOUT 180)
endif()

ament_package()
```

## 4. Testing Tools Cheat Sheet

| Language | Framework               | ROS 2 Integration      | Typical Use Case                 |
| -------- | ----------------------- | ---------------------- | -------------------------------- |
| C++      | GoogleTest + GoogleMock | ament_add_gtest        | Pure logic + mocked hardware     |
| C++      | Catch2                  | catch_ros2 (community) | Lighter alternative              |
| Python   | pytest                  | ament_add_pytest_test  | Node logic, parameters, fixtures |
| Any      | launch_testing          | add_launch_test        | Full node lifecycle & topics     |

## 5. Example Tests

### 5.1 Pure Logic Unit Test (C++)

C++

```
// test/gtest/test_path_planner.cpp
#include <gtest/gtest.h>
#include "my_package/path_planner.hpp"

TEST(PathPlanner, StraightLine) {
  PathPlanner planner;
  auto path = planner.plan({0,0}, {10,0});
  EXPECT_EQ(path.size(), 11);
  EXPECT_DOUBLE_EQ(path.back().pose.position.x, 10.0);
}
```

### 5.2 Hardware Mock with GoogleMock (C++)

C++

```
// test/mocks/mock_lidar.hpp
class MockLidar : public LidarInterface {
 public:
  MOCK_METHOD(std::vector<float>, scan, (), (override));
};

// test/gtest/test_avoidance.cpp
TEST(AvoidanceNode, StopsWhenObjectClose) {
  MockLidar lidar;
  EXPECT_CALL(lidar, scan())
    .WillOnce(Return(std::vector<float>(360, 10.0f)))  // far
    .WillOnce(Return(std::vector<float>(360, 0.2f)));  // too close

  AvoidanceNode node(std::make_unique<MockLidar>(lidar));
  node.spin_some(100ms);
  EXPECT_NEAR(node.get_linear_velocity(), 0.0, 1e-6);
}
```

### 5.3 Integration Launch Test (Python)

Python

```
# test/launch/navigator_test.launch.py
import launch
import launch_ros.actions
from launch_testing.actions import ReadyToTest

def generate_test_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='perseus_navigation',
            executable='navigator_node',
            parameters=[{'use_sim_time': True}]
        ),
        ReadyToTest()
    ])
```

## 6. Running Tests Locally

Bash

```
colcon test --packages-select perseus_navigation perseus_hardware
colcon test-result --verbose

# Python coverage
colcon test --pytest-args "--cov=perseus_navigation --cov-report=html"

# C++ coverage
colcon test --cmake-args -DCODE_COVERAGE=ON
lcov --capture --directory . --output-file coverage.info
genhtml coverage.info --output-directory coverage_html
```

## 7. GitHub Actions Snippet

YAML

```
- name: Run ROS 2 tests
  run: |
    colcon test --ctest-args tests --pytest-args -v
    colcon test-result --all
```

## 8. Hardware-in-the-Loop Tips

- Use ros2_control mock_components or fake_components in CI
- Emulate CAN/EtherCAT inside Docker when needed
- Keep real hardware tests in a separate nightly/manual job

## 9. Further Reading

- Official ROS 2 testing guide → https://docs.ros.org/en/rolling/Tutorials/Intermediate/Testing/Testing-Main.html
- launch_testing examples → https://github.com/ros2/launch_ros/tree/rolling/launch_testing
- Real-world repo with excellent CI → https://github.com/ros-industrial/universal_robot
- GTest + ROS 2 tutorial → https://automaticaddison.com/how-to-create-unit-tests-with-gtest-ros-2-jazzy/

