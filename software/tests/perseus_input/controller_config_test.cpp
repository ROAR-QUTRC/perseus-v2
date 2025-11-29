#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <regex>
#include <string>
#include <unordered_set>
#include <vector>

class ControllerConfigTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Config files are in perseus_input_config package
        config_dir = "../../src/perseus_input_config/config/";
        config_files = {
            "xbox_controller_wireless.yaml",
            "xbox_controller_wired.yaml",
            "logitech_controller.yaml",
            "8bitdo_controller.yaml"};
    }

    std::string config_dir;
    std::vector<std::string> config_files;

    YAML::Node loadConfig(const std::string& filename)
    {
        return YAML::LoadFile(config_dir + filename);
    }

    bool hasParam(const YAML::Node& node, const std::string& path)
    {
        std::vector<std::string> parts;
        std::stringstream ss(path);
        std::string part;
        while (std::getline(ss, part, '.'))
        {
            parts.push_back(part);
        }

        YAML::Node current = node;
        for (const auto& p : parts)
        {
            if (!current[p])
                return false;
            current = current[p];
        }
        return true;
    }

    template <typename T>
    T getParam(const YAML::Node& node, const std::string& path)
    {
        std::vector<std::string> parts;
        std::stringstream ss(path);
        std::string part;
        while (std::getline(ss, part, '.'))
        {
            parts.push_back(part);
        }

        YAML::Node current = node;
        for (const auto& p : parts)
        {
            current = current[p];
        }
        return current.as<T>();
    }
};

TEST_F(ControllerConfigTest, AllConfigFilesExist)
{
    for (const auto& file : config_files)
    {
        std::ifstream f(config_dir + file);
        EXPECT_TRUE(f.good()) << "Config file not found: " << file;
    }
}

TEST_F(ControllerConfigTest, AllConfigFilesAreValidYaml)
{
    for (const auto& file : config_files)
    {
        EXPECT_NO_THROW({
            YAML::LoadFile(config_dir + file);
        }) << "Invalid YAML in: "
           << file;
    }
}

TEST_F(ControllerConfigTest, AllConfigsHaveGenericControllerSection)
{
    for (const auto& file : config_files)
    {
        YAML::Node config = loadConfig(file);
        EXPECT_TRUE(config["generic_controller"]) << "Missing generic_controller section in: " << file;
        EXPECT_TRUE(config["generic_controller"]["ros__parameters"])
            << "Missing ros__parameters in: " << file;
    }
}

TEST_F(ControllerConfigTest, AllConfigsHaveDriveSection)
{
    for (const auto& file : config_files)
    {
        YAML::Node config = loadConfig(file);
        YAML::Node params = config["generic_controller"]["ros__parameters"];

        EXPECT_TRUE(params["drive"]) << "Missing drive section in: " << file;
        EXPECT_TRUE(params["drive"]["forward"]) << "Missing drive.forward in: " << file;
        EXPECT_TRUE(params["drive"]["turn"]) << "Missing drive.turn in: " << file;
    }
}

TEST_F(ControllerConfigTest, DriveAxesHaveValidConfiguration)
{
    for (const auto& file : config_files)
    {
        YAML::Node config = loadConfig(file);
        YAML::Node params = config["generic_controller"]["ros__parameters"];

        // Forward axis
        YAML::Node forward = params["drive"]["forward"];
        if (forward["axis"])
        {
            int axis = forward["axis"].as<int>();
            EXPECT_GE(axis, -1) << "Invalid forward axis in: " << file;
            EXPECT_LE(axis, 7) << "Forward axis out of range in: " << file;
        }

        // Turn axis
        YAML::Node turn = params["drive"]["turn"];
        if (turn["axis"])
        {
            int axis = turn["axis"].as<int>();
            EXPECT_GE(axis, -1) << "Invalid turn axis in: " << file;
            EXPECT_LE(axis, 7) << "Turn axis out of range in: " << file;
        }
    }
}

TEST_F(ControllerConfigTest, ScalingValuesAreReasonable)
{
    for (const auto& file : config_files)
    {
        YAML::Node config = loadConfig(file);
        YAML::Node params = config["generic_controller"]["ros__parameters"];

        // Check drive scaling
        if (params["drive"]["forward"]["scaling"])
        {
            double scaling = params["drive"]["forward"]["scaling"].as<double>();
            EXPECT_GE(scaling, -10.0) << "Forward scaling too negative in: " << file;
            EXPECT_LE(scaling, 10.0) << "Forward scaling too large in: " << file;
        }

        if (params["drive"]["turn"]["scaling"])
        {
            double scaling = params["drive"]["turn"]["scaling"].as<double>();
            EXPECT_GE(scaling, -10.0) << "Turn scaling too negative in: " << file;
            EXPECT_LE(scaling, 10.0) << "Turn scaling too large in: " << file;
        }
    }
}

TEST_F(ControllerConfigTest, EnableConfigurationsAreValid)
{
    for (const auto& file : config_files)
    {
        YAML::Node config = loadConfig(file);
        YAML::Node params = config["generic_controller"]["ros__parameters"];

        YAML::Node forward = params["drive"]["forward"];
        if (forward["enable"])
        {
            YAML::Node enable = forward["enable"];
            // Must have either axis or follows
            bool hasAxis = enable["axis"].IsDefined();
            bool hasFollows = enable["follows"].IsDefined();
            EXPECT_TRUE(hasAxis || hasFollows)
                << "Enable must have axis or follows in: " << file;

            if (hasAxis)
            {
                int axis = enable["axis"].as<int>();
                EXPECT_GE(axis, 0) << "Invalid enable axis in: " << file;
                EXPECT_LE(axis, 7) << "Enable axis out of range in: " << file;
            }
        }
    }
}

TEST_F(ControllerConfigTest, TimeoutIsConfigured)
{
    for (const auto& file : config_files)
    {
        YAML::Node config = loadConfig(file);
        YAML::Node params = config["generic_controller"]["ros__parameters"];

        EXPECT_TRUE(params["timeout_ns"]) << "Missing timeout_ns in: " << file;
        if (params["timeout_ns"])
        {
            int64_t timeout = params["timeout_ns"].as<int64_t>();
            EXPECT_GT(timeout, 0) << "Timeout must be positive in: " << file;
            EXPECT_LT(timeout, 10000000000LL) << "Timeout too large (>10s) in: " << file;
        }
    }
}

TEST_F(ControllerConfigTest, ButtonIndicesAreValid)
{
    for (const auto& file : config_files)
    {
        YAML::Node config = loadConfig(file);
        YAML::Node params = config["generic_controller"]["ros__parameters"];

        // Check bucket jaws buttons if present
        if (params["bucket"] && params["bucket"]["jaws"])
        {
            YAML::Node jaws = params["bucket"]["jaws"];
            if (jaws["button_positive"])
            {
                int button = jaws["button_positive"].as<int>();
                EXPECT_GE(button, -1) << "Invalid button_positive in: " << file;
                EXPECT_LE(button, 15) << "button_positive out of range in: " << file;
            }
            if (jaws["button_negative"])
            {
                int button = jaws["button_negative"].as<int>();
                EXPECT_GE(button, -1) << "Invalid button_negative in: " << file;
                EXPECT_LE(button, 15) << "button_negative out of range in: " << file;
            }
        }
    }
}

TEST_F(ControllerConfigTest, BucketSectionIsConsistent)
{
    for (const auto& file : config_files)
    {
        YAML::Node config = loadConfig(file);
        YAML::Node params = config["generic_controller"]["ros__parameters"];

        if (params["bucket"])
        {
            YAML::Node bucket = params["bucket"];
            // If bucket section exists, it should have expected actuators
            std::vector<std::string> expectedActuators = {"lift", "tilt", "jaws", "rotate", "magnet"};
            for (const auto& actuator : expectedActuators)
            {
                EXPECT_TRUE(bucket[actuator])
                    << "Missing bucket." << actuator << " in: " << file;
            }
        }
    }
}

TEST_F(ControllerConfigTest, FollowsReferencesAreValid)
{
    for (const auto& file : config_files)
    {
        YAML::Node config = loadConfig(file);
        YAML::Node params = config["generic_controller"]["ros__parameters"];

        // Check turn enable follows reference
        if (params["drive"]["turn"]["enable"]["follows"])
        {
            std::string follows = params["drive"]["turn"]["enable"]["follows"].as<std::string>();
            // The follows reference should point to a valid path
            EXPECT_FALSE(follows.empty()) << "Empty follows reference in: " << file;
            // Common pattern: "drive.forward.enable"
            EXPECT_TRUE(follows.find("drive.forward.enable") != std::string::npos ||
                        follows.find("bucket.") != std::string::npos)
                << "Unexpected follows reference '" << follows << "' in: " << file;
        }
    }
}

TEST_F(ControllerConfigTest, TurboScalingIsReasonable)
{
    for (const auto& file : config_files)
    {
        YAML::Node config = loadConfig(file);
        YAML::Node params = config["generic_controller"]["ros__parameters"];

        if (params["drive"]["forward"]["turbo"])
        {
            double turbo = params["drive"]["forward"]["turbo"].as<double>();
            EXPECT_GT(turbo, 0.0) << "Turbo scaling should be positive in: " << file;
            EXPECT_LE(turbo, 10.0) << "Turbo scaling too large in: " << file;
        }
    }
}
