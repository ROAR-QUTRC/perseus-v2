#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <regex>
#include <string>
#include <unordered_set>
#include <vector>

class GzBridgeConfigTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        config_file_path = "../../src/perseus_simulation/config/gz_bridge.yaml";
    }

    std::string config_file_path;

    struct BridgeMapping
    {
        std::string ros_topic_name;
        std::string gz_topic_name;
        std::string ros_type_name;
        std::string gz_type_name;
        std::string direction;
    };

    std::vector<BridgeMapping> loadBridgeMappings()
    {
        std::vector<BridgeMapping> mappings;

        try
        {
            YAML::Node config = YAML::LoadFile(config_file_path);

            for (const auto& mapping : config)
            {
                BridgeMapping bridge_mapping;
                bridge_mapping.ros_topic_name = mapping["ros_topic_name"].as<std::string>();
                bridge_mapping.gz_topic_name = mapping["gz_topic_name"].as<std::string>();
                bridge_mapping.ros_type_name = mapping["ros_type_name"].as<std::string>();
                bridge_mapping.gz_type_name = mapping["gz_type_name"].as<std::string>();
                bridge_mapping.direction = mapping["direction"].as<std::string>();

                mappings.push_back(bridge_mapping);
            }
        }
        catch (const YAML::Exception& e)
        {
            ADD_FAILURE() << "Failed to load YAML config: " << e.what();
            return mappings;
        }

        return mappings;
    }
};

TEST_F(GzBridgeConfigTest, ConfigFileExists)
{
    std::ifstream file(config_file_path);
    ASSERT_TRUE(file.good()) << "gz_bridge.yaml config file not found at: " << config_file_path;
}

TEST_F(GzBridgeConfigTest, ValidYamlStructure)
{
    auto mappings = loadBridgeMappings();
    ASSERT_GT(mappings.size(), 0) << "No bridge mappings found in config file";

    for (const auto& mapping : mappings)
    {
        EXPECT_FALSE(mapping.ros_topic_name.empty()) << "Empty ros_topic_name found";
        EXPECT_FALSE(mapping.gz_topic_name.empty()) << "Empty gz_topic_name found";
        EXPECT_FALSE(mapping.ros_type_name.empty()) << "Empty ros_type_name found";
        EXPECT_FALSE(mapping.gz_type_name.empty()) << "Empty gz_type_name found";
        EXPECT_FALSE(mapping.direction.empty()) << "Empty direction found";
    }
}

TEST_F(GzBridgeConfigTest, ValidDirections)
{
    auto mappings = loadBridgeMappings();
    std::unordered_set<std::string> valid_directions = {
        "ROS_TO_GZ", "GZ_TO_ROS", "BIDIRECTIONAL"};

    for (const auto& mapping : mappings)
    {
        EXPECT_TRUE(valid_directions.count(mapping.direction) > 0)
            << "Invalid direction '" << mapping.direction
            << "' for topic '" << mapping.ros_topic_name << "'";
    }
}

TEST_F(GzBridgeConfigTest, ValidRosMessageTypes)
{
    auto mappings = loadBridgeMappings();
    // ROS 2 message types follow the pattern: package_name/msg/MessageName
    // package_name: lowercase with underscores
    // MessageName: PascalCase
    std::regex ros_type_pattern("^[a-z0-9_]+/msg/[A-Z][A-Za-z0-9]*$");

    for (const auto& mapping : mappings)
    {
        EXPECT_TRUE(std::regex_match(mapping.ros_type_name, ros_type_pattern))
            << "Invalid ROS message type format: '" << mapping.ros_type_name
            << "' for topic '" << mapping.ros_topic_name << "'. "
            << "Expected format: package_name/msg/MessageName";
    }
}

TEST_F(GzBridgeConfigTest, ValidGazeboMessageTypes)
{
    auto mappings = loadBridgeMappings();
    // Gazebo message types follow the pattern: gz.msgs.MessageName
    // MessageName: PascalCase, may contain underscores
    std::regex gz_type_pattern("^gz\\.msgs\\.[A-Z][A-Za-z0-9_]*$");

    for (const auto& mapping : mappings)
    {
        EXPECT_TRUE(std::regex_match(mapping.gz_type_name, gz_type_pattern))
            << "Invalid Gazebo message type format: '" << mapping.gz_type_name
            << "' for topic '" << mapping.ros_topic_name << "'. "
            << "Expected format: gz.msgs.MessageName";
    }
}

TEST_F(GzBridgeConfigTest, UniqueTopicNames)
{
    auto mappings = loadBridgeMappings();
    std::unordered_set<std::string> ros_topics;
    std::unordered_set<std::string> gz_topics;

    for (const auto& mapping : mappings)
    {
        EXPECT_TRUE(ros_topics.insert(mapping.ros_topic_name).second)
            << "Duplicate ROS topic name: " << mapping.ros_topic_name;
        EXPECT_TRUE(gz_topics.insert(mapping.gz_topic_name).second)
            << "Duplicate Gazebo topic name: " << mapping.gz_topic_name;
    }
}

TEST_F(GzBridgeConfigTest, ExpectedCoreTopics)
{
    auto mappings = loadBridgeMappings();
    std::unordered_set<std::string> required_topics = {
        "clock", "tf", "odom", "joint_states"};

    std::unordered_set<std::string> found_topics;
    for (const auto& mapping : mappings)
    {
        found_topics.insert(mapping.ros_topic_name);
    }

    for (const auto& required_topic : required_topics)
    {
        EXPECT_TRUE(found_topics.count(required_topic) > 0)
            << "Required topic '" << required_topic << "' not found in bridge configuration";
    }
}