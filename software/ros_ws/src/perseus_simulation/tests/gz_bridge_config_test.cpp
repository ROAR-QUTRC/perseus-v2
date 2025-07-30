#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <fstream>
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
    std::unordered_set<std::string> known_ros_types = {
        "rosgraph_msgs/msg/Clock",
        "tf2_msgs/msg/TFMessage",
        "nav_msgs/msg/Odometry",
        "sensor_msgs/msg/JointState",
        "sensor_msgs/msg/LaserScan",
        "sensor_msgs/msg/Imu"};

    for (const auto& mapping : mappings)
    {
        EXPECT_TRUE(known_ros_types.count(mapping.ros_type_name) > 0)
            << "Unknown ROS message type '" << mapping.ros_type_name
            << "' for topic '" << mapping.ros_topic_name << "'";
    }
}

TEST_F(GzBridgeConfigTest, ValidGazeboMessageTypes)
{
    auto mappings = loadBridgeMappings();
    std::unordered_set<std::string> known_gz_types = {
        "gz.msgs.Clock",
        "gz.msgs.Pose_V",
        "gz.msgs.Odometry",
        "gz.msgs.Model",
        "gz.msgs.LaserScan",
        "gz.msgs.IMU"};

    for (const auto& mapping : mappings)
    {
        EXPECT_TRUE(known_gz_types.count(mapping.gz_type_name) > 0)
            << "Unknown Gazebo message type '" << mapping.gz_type_name
            << "' for topic '" << mapping.ros_topic_name << "'";
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