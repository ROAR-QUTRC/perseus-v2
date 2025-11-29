#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <fstream>
#include <string>
#include <unordered_set>
#include <vector>

class PerseusLiteControllerConfigTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        config_file_path = "../../src/perseus_lite/config/perseus_lite_controllers.yaml";
    }

    std::string config_file_path;

    YAML::Node loadConfig()
    {
        return YAML::LoadFile(config_file_path);
    }
};

TEST_F(PerseusLiteControllerConfigTest, ConfigFileExists)
{
    std::ifstream file(config_file_path);
    ASSERT_TRUE(file.good()) << "perseus_lite_controllers.yaml not found at: " << config_file_path;
}

TEST_F(PerseusLiteControllerConfigTest, ValidYamlStructure)
{
    EXPECT_NO_THROW({
        YAML::LoadFile(config_file_path);
    }) << "Invalid YAML in perseus_lite_controllers.yaml";
}

TEST_F(PerseusLiteControllerConfigTest, HasControllerManagerSection)
{
    YAML::Node config = loadConfig();
    EXPECT_TRUE(config["controller_manager"]) << "Missing controller_manager section";
    EXPECT_TRUE(config["controller_manager"]["ros__parameters"])
        << "Missing controller_manager ros__parameters";
}

TEST_F(PerseusLiteControllerConfigTest, ControllerManagerUpdateRateIsValid)
{
    YAML::Node config = loadConfig();
    YAML::Node params = config["controller_manager"]["ros__parameters"];

    ASSERT_TRUE(params["update_rate"]) << "Missing update_rate parameter";
    int updateRate = params["update_rate"].as<int>();
    EXPECT_GT(updateRate, 0) << "Update rate must be positive";
    EXPECT_LE(updateRate, 1000) << "Update rate too high (>1000 Hz)";
}

TEST_F(PerseusLiteControllerConfigTest, HasRequiredControllers)
{
    YAML::Node config = loadConfig();
    YAML::Node params = config["controller_manager"]["ros__parameters"];

    // Check joint_state_broadcaster is defined
    EXPECT_TRUE(params["joint_state_broadcaster"])
        << "Missing joint_state_broadcaster controller definition";
    EXPECT_TRUE(params["joint_state_broadcaster"]["type"])
        << "Missing joint_state_broadcaster type";

    // Check diff_drive_base_controller is defined
    EXPECT_TRUE(params["diff_drive_base_controller"])
        << "Missing diff_drive_base_controller definition";
    EXPECT_TRUE(params["diff_drive_base_controller"]["type"])
        << "Missing diff_drive_base_controller type";
}

TEST_F(PerseusLiteControllerConfigTest, ControllerTypesAreCorrect)
{
    YAML::Node config = loadConfig();
    YAML::Node params = config["controller_manager"]["ros__parameters"];

    std::string jsbType = params["joint_state_broadcaster"]["type"].as<std::string>();
    EXPECT_EQ("joint_state_broadcaster/JointStateBroadcaster", jsbType)
        << "Incorrect joint_state_broadcaster type";

    std::string ddcType = params["diff_drive_base_controller"]["type"].as<std::string>();
    EXPECT_EQ("diff_drive_controller/DiffDriveController", ddcType)
        << "Incorrect diff_drive_base_controller type";
}

TEST_F(PerseusLiteControllerConfigTest, HasHardwareInterfaceSection)
{
    YAML::Node config = loadConfig();
    EXPECT_TRUE(config["Perseus_Lite"]) << "Missing Perseus_Lite hardware interface section";
    EXPECT_TRUE(config["Perseus_Lite"]["ros__parameters"])
        << "Missing Perseus_Lite ros__parameters";
}

TEST_F(PerseusLiteControllerConfigTest, AllFourWheelJointsDefined)
{
    YAML::Node config = loadConfig();
    YAML::Node joints = config["Perseus_Lite"]["ros__parameters"]["joints"];

    ASSERT_TRUE(joints) << "Missing joints section in hardware interface";

    std::vector<std::string> expectedJoints = {
        "front_left_wheel_joint",
        "front_right_wheel_joint",
        "rear_left_wheel_joint",
        "rear_right_wheel_joint"};

    for (const auto& joint : expectedJoints)
    {
        EXPECT_TRUE(joints[joint]) << "Missing joint: " << joint;
    }
}

TEST_F(PerseusLiteControllerConfigTest, JointsHaveRequiredInterfaces)
{
    YAML::Node config = loadConfig();
    YAML::Node joints = config["Perseus_Lite"]["ros__parameters"]["joints"];

    std::vector<std::string> jointNames = {
        "front_left_wheel_joint",
        "front_right_wheel_joint",
        "rear_left_wheel_joint",
        "rear_right_wheel_joint"};

    for (const auto& jointName : jointNames)
    {
        YAML::Node joint = joints[jointName];
        ASSERT_TRUE(joint) << "Missing joint: " << jointName;

        // Check command interfaces
        EXPECT_TRUE(joint["command_interfaces"])
            << "Missing command_interfaces for: " << jointName;

        // Check state interfaces
        EXPECT_TRUE(joint["state_interfaces"])
            << "Missing state_interfaces for: " << jointName;

        // Verify velocity is in command interfaces
        bool hasVelocityCommand = false;
        for (const auto& iface : joint["command_interfaces"])
        {
            if (iface.as<std::string>() == "velocity")
                hasVelocityCommand = true;
        }
        EXPECT_TRUE(hasVelocityCommand)
            << "Missing velocity command interface for: " << jointName;
    }
}

TEST_F(PerseusLiteControllerConfigTest, HasDiffDriveControllerSection)
{
    YAML::Node config = loadConfig();
    EXPECT_TRUE(config["diff_drive_base_controller"])
        << "Missing diff_drive_base_controller section";
    EXPECT_TRUE(config["diff_drive_base_controller"]["ros__parameters"])
        << "Missing diff_drive_base_controller ros__parameters";
}

TEST_F(PerseusLiteControllerConfigTest, WheelNamesMatchHardwareJoints)
{
    YAML::Node config = loadConfig();
    YAML::Node ddcParams = config["diff_drive_base_controller"]["ros__parameters"];
    YAML::Node hwJoints = config["Perseus_Lite"]["ros__parameters"]["joints"];

    // Get hardware joint names
    std::unordered_set<std::string> hwJointNames;
    for (const auto& joint : hwJoints)
    {
        hwJointNames.insert(joint.first.as<std::string>());
    }

    // Check left wheel names
    ASSERT_TRUE(ddcParams["left_wheel_names"]) << "Missing left_wheel_names";
    for (const auto& wheel : ddcParams["left_wheel_names"])
    {
        std::string name = wheel.as<std::string>();
        EXPECT_TRUE(hwJointNames.count(name) > 0)
            << "Left wheel '" << name << "' not found in hardware joints";
    }

    // Check right wheel names
    ASSERT_TRUE(ddcParams["right_wheel_names"]) << "Missing right_wheel_names";
    for (const auto& wheel : ddcParams["right_wheel_names"])
    {
        std::string name = wheel.as<std::string>();
        EXPECT_TRUE(hwJointNames.count(name) > 0)
            << "Right wheel '" << name << "' not found in hardware joints";
    }
}

TEST_F(PerseusLiteControllerConfigTest, WheelGeometryIsValid)
{
    YAML::Node config = loadConfig();
    YAML::Node params = config["diff_drive_base_controller"]["ros__parameters"];

    // Wheel separation
    ASSERT_TRUE(params["wheel_separation"]) << "Missing wheel_separation";
    double wheelSep = params["wheel_separation"].as<double>();
    EXPECT_GT(wheelSep, 0.0) << "Wheel separation must be positive";
    EXPECT_LT(wheelSep, 5.0) << "Wheel separation too large (>5m)";

    // Wheel radius
    ASSERT_TRUE(params["wheel_radius"]) << "Missing wheel_radius";
    double wheelRadius = params["wheel_radius"].as<double>();
    EXPECT_GT(wheelRadius, 0.0) << "Wheel radius must be positive";
    EXPECT_LT(wheelRadius, 1.0) << "Wheel radius too large (>1m)";

    // Wheels per side
    ASSERT_TRUE(params["wheels_per_side"]) << "Missing wheels_per_side";
    int wheelsPerSide = params["wheels_per_side"].as<int>();
    EXPECT_GT(wheelsPerSide, 0) << "Wheels per side must be positive";
    EXPECT_LE(wheelsPerSide, 4) << "Too many wheels per side (>4)";
}

TEST_F(PerseusLiteControllerConfigTest, KinematicLimitsAreValid)
{
    YAML::Node config = loadConfig();
    YAML::Node params = config["diff_drive_base_controller"]["ros__parameters"];

    // Linear velocity limit
    if (params["linear.x.max_velocity"])
    {
        double maxLinVel = params["linear.x.max_velocity"].as<double>();
        EXPECT_GT(maxLinVel, 0.0) << "Max linear velocity must be positive";
        EXPECT_LE(maxLinVel, 10.0) << "Max linear velocity too high (>10 m/s)";
    }

    // Linear acceleration limit
    if (params["linear.x.max_acceleration"])
    {
        double maxLinAcc = params["linear.x.max_acceleration"].as<double>();
        EXPECT_GT(maxLinAcc, 0.0) << "Max linear acceleration must be positive";
        EXPECT_LE(maxLinAcc, 50.0) << "Max linear acceleration too high (>50 m/s^2)";
    }

    // Angular velocity limit
    if (params["angular.z.max_velocity"])
    {
        double maxAngVel = params["angular.z.max_velocity"].as<double>();
        EXPECT_GT(maxAngVel, 0.0) << "Max angular velocity must be positive";
        EXPECT_LE(maxAngVel, 20.0) << "Max angular velocity too high (>20 rad/s)";
    }

    // Angular acceleration limit
    if (params["angular.z.max_acceleration"])
    {
        double maxAngAcc = params["angular.z.max_acceleration"].as<double>();
        EXPECT_GT(maxAngAcc, 0.0) << "Max angular acceleration must be positive";
        EXPECT_LE(maxAngAcc, 100.0) << "Max angular acceleration too high (>100 rad/s^2)";
    }
}

TEST_F(PerseusLiteControllerConfigTest, CmdVelTimeoutIsValid)
{
    YAML::Node config = loadConfig();
    YAML::Node params = config["diff_drive_base_controller"]["ros__parameters"];

    ASSERT_TRUE(params["cmd_vel_timeout"]) << "Missing cmd_vel_timeout";
    double timeout = params["cmd_vel_timeout"].as<double>();
    EXPECT_GT(timeout, 0.0) << "Cmd vel timeout must be positive";
    EXPECT_LE(timeout, 10.0) << "Cmd vel timeout too large (>10s)";
}

TEST_F(PerseusLiteControllerConfigTest, OdomFrameIdsAreConfigured)
{
    YAML::Node config = loadConfig();
    YAML::Node params = config["diff_drive_base_controller"]["ros__parameters"];

    EXPECT_TRUE(params["odom_frame_id"]) << "Missing odom_frame_id";
    EXPECT_TRUE(params["base_frame_id"]) << "Missing base_frame_id";

    std::string odomFrame = params["odom_frame_id"].as<std::string>();
    std::string baseFrame = params["base_frame_id"].as<std::string>();

    EXPECT_FALSE(odomFrame.empty()) << "odom_frame_id is empty";
    EXPECT_FALSE(baseFrame.empty()) << "base_frame_id is empty";
    EXPECT_NE(odomFrame, baseFrame) << "odom_frame_id and base_frame_id should be different";
}

TEST_F(PerseusLiteControllerConfigTest, CovarianceDiagonalsAreValid)
{
    YAML::Node config = loadConfig();
    YAML::Node params = config["diff_drive_base_controller"]["ros__parameters"];

    // Check pose covariance diagonal
    if (params["pose_covariance_diagonal"])
    {
        std::vector<double> poseCov = params["pose_covariance_diagonal"].as<std::vector<double>>();
        EXPECT_EQ(6u, poseCov.size()) << "pose_covariance_diagonal should have 6 elements";
        for (size_t i = 0; i < poseCov.size(); i++)
        {
            EXPECT_GE(poseCov[i], 0.0)
                << "pose_covariance_diagonal[" << i << "] should be non-negative";
        }
    }

    // Check twist covariance diagonal
    if (params["twist_covariance_diagonal"])
    {
        std::vector<double> twistCov = params["twist_covariance_diagonal"].as<std::vector<double>>();
        EXPECT_EQ(6u, twistCov.size()) << "twist_covariance_diagonal should have 6 elements";
        for (size_t i = 0; i < twistCov.size(); i++)
        {
            EXPECT_GE(twistCov[i], 0.0)
                << "twist_covariance_diagonal[" << i << "] should be non-negative";
        }
    }
}

TEST_F(PerseusLiteControllerConfigTest, PublishRateIsValid)
{
    YAML::Node config = loadConfig();
    YAML::Node params = config["diff_drive_base_controller"]["ros__parameters"];

    ASSERT_TRUE(params["publish_rate"]) << "Missing publish_rate";
    double publishRate = params["publish_rate"].as<double>();
    EXPECT_GT(publishRate, 0.0) << "Publish rate must be positive";
    EXPECT_LE(publishRate, 1000.0) << "Publish rate too high (>1000 Hz)";
}

TEST_F(PerseusLiteControllerConfigTest, WriteOpModesIsConfigured)
{
    YAML::Node config = loadConfig();
    YAML::Node params = config["diff_drive_base_controller"]["ros__parameters"];

    ASSERT_TRUE(params["write_op_modes"]) << "Missing write_op_modes";
    std::vector<std::string> opModes = params["write_op_modes"].as<std::vector<std::string>>();
    EXPECT_FALSE(opModes.empty()) << "write_op_modes should not be empty";

    // Should contain "velocity" for diff drive
    bool hasVelocity = false;
    for (const auto& mode : opModes)
    {
        if (mode == "velocity")
            hasVelocity = true;
    }
    EXPECT_TRUE(hasVelocity) << "write_op_modes should contain 'velocity'";
}

// Regression test: ensure specific known-good values haven't changed accidentally
TEST_F(PerseusLiteControllerConfigTest, RegressionWheelGeometry)
{
    YAML::Node config = loadConfig();
    YAML::Node params = config["diff_drive_base_controller"]["ros__parameters"];

    // These are the expected values for Perseus Lite
    double wheelSep = params["wheel_separation"].as<double>();
    double wheelRadius = params["wheel_radius"].as<double>();

    // Warn if values have changed from expected
    EXPECT_NEAR(0.40, wheelSep, 0.01)
        << "Warning: wheel_separation changed from expected 0.40m";
    EXPECT_NEAR(0.075, wheelRadius, 0.01)
        << "Warning: wheel_radius changed from expected 0.075m";
}
