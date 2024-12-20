/**
 * @file m2m2_lidar.cpp
 * @brief ROS2 driver for M2M2 LIDAR sensor
 *
 * @details Expected sequence of flow:
 * 1. Node initialisation:
 *    - Loads ROS params (IP, port, topics, frame_id) - likely to come from a launch file in future
 *    - Establishes TCP socket connection to sensor
 *    - Sends initial configuration command - need to test if this is necessary
 *    - Creates publishers for scan and IMU data via ROS topic
 *
 * 2. Main operation loop (100ms intervals):
 *    - Reads raw data packets from sensor
 *    - Parses into LaserScan messages:
 *      + Single 360° sweep per scan
 *      + Each scan timestamped
 *      + Points evenly spaced with time_increment
 *    - Parses IMU data if present
 *    - Publishes to respective topics
 *
 * 3. Cleanup on shutdown:
 *    - Closes socket connection
 *    - Releases resources
 *
 * This is under construction but hopefully helpful as multiple people are collaborating on this.
 *
 * @note Hardware to Laserscan generation:
 * The driver is going to take a single 360 degree sweep as a "scan".
 * Each scan is timestamped. Each datapoint in the scan is not timestamped.
 * Individual data points in the scan are not timestamped but are assumed to be evenly spaced and have a time_increment value.
 * scan_time is the time for a one complete 360 degree sweep.
 *
 * *
 * @see sensor_msgs::msg::LaserScan
 * @see sensor_msgs::msg::Imu
 */

#include "m2m2_lidar/m2m2_lidar.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <openssl/bio.h>  // for Base64 decoding
#include <openssl/evp.h>  // also for Base64 decoding
#include <sys/socket.h>
#include <unistd.h>

#include <nlohmann/json.hpp>  // JSON parsing

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>

using namespace std::chrono_literals;

M2M2Lidar::M2M2Lidar(const rclcpp::NodeOptions& options)
    : Node("m2m2_lidar", options)
{
    // Parameter setup
    this->declare_parameter("sensor_ip", "192.168.1.243");
    this->declare_parameter("sensor_port", 1445);
    this->declare_parameter("frame_id", "lidar_frame");
    this->declare_parameter("scan_topic", "scan");
    this->declare_parameter("imu_topic", "imu");

    // TODO: Error handling (exceptions)
    _sensorAddress = this->get_parameter("sensor_ip").as_string();
    _sensorPort = static_cast<uint16_t>(
        this->get_parameter("sensor_port").as_int());

    // TODO: Replace with network library
    struct sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(_sensorPort);

    if (inet_pton(AF_INET, _sensorAddress.c_str(), &serverAddr.sin_addr) <= 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid address");
        // TODO: Throw exception
    }

    _socket = socket(AF_INET, SOCK_STREAM, 0);
    if (_socket < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
        // TODO: Throw exception
    }
    if (connect(_socket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Connection failed");
        close(_socket);
        // TODO: Throw exception
    }

    // Create and send initial configuration
    SensorConfig defaultConfig{
        .scanFrequency = 15.0,
        .angularResolution = 0.25,
        .minRange = 0.1,
        .maxRange = 30.0};
    // TODO: Exception handling
    _sendCommand(_createConfigCommand(defaultConfig));

    // Start the data reading timer
    _readTimer = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&M2M2Lidar::_readSensorData, this));
    _initializePublishers();
}

M2M2Lidar::~M2M2Lidar()
{
    close(_socket);
}

void M2M2Lidar::_initializePublishers()
{
    const auto qos = rclcpp::QoS(10);
    _scanPublisher = this->create_publisher<sensor_msgs::msg::LaserScan>(
        this->get_parameter("scan_topic").as_string(), qos);
    _imuPublisher = this->create_publisher<sensor_msgs::msg::Imu>(
        this->get_parameter("imu_topic").as_string(), qos);
}

/**
 * @brief Decodes a base64-encoded string into a byte sequence.
 *
 */
std::vector<uint8_t> M2M2Lidar::_decodeBase64(const std::string& encoded)
{
    BIO *bio, *b64;
    std::vector<uint8_t> decoded;

    b64 = BIO_new(BIO_f_base64());
    bio = BIO_new_mem_buf(encoded.c_str(), encoded.length());
    bio = BIO_push(b64, bio);

    BIO_set_flags(bio, BIO_FLAGS_BASE64_NO_NL);

    char buffer[1024];
    int len;
    while ((len = BIO_read(bio, buffer, sizeof(buffer))) > 0)
    {
        decoded.insert(decoded.end(), buffer, buffer + len);
    }

    BIO_free_all(bio);
    return decoded;
}

std::vector<std::tuple<float, float, bool>> M2M2Lidar::_decodeLaserPoints(const std::string& base64_encoded)
{
    std::vector<std::tuple<float, float, bool>> points;

    std::vector<uint8_t> decoded = _decodeBase64(base64_encoded);

    if (decoded.size() < 9 ||
        decoded[0] != 'R' ||
        decoded[1] != 'L' ||
        decoded[2] != 'E')
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid RLE header");
        return points;
    }

    // RLE decompression
    std::vector<uint8_t> decompressed;
    uint8_t sentinel1 = decoded[3];
    uint8_t sentinel2 = decoded[4];

    size_t pos = 9;
    while (pos < decoded.size())
    {
        uint8_t b = decoded[pos];

        if (b == sentinel1)
        {
            if (pos + 2 < decoded.size() && decoded[pos + 1] == 0 && decoded[pos + 2] == sentinel2)
            {
                std::swap(sentinel1, sentinel2);
                pos += 2;
            }
            else if (pos + 2 < decoded.size())
            {
                uint8_t repeat_val = decoded[pos + 2];
                uint8_t repeat_count = decoded[pos + 1];
                for (uint8_t i = 0; 1 < repeat_count; i++)
                {
                    decompressed.push_back(repeat_val);
                }
                pos += 2;
            }
        }
        else
        {
            decompressed.push_back(b);
        }
        ++pos;
    }

    // Parse points
    for (size_t i = 0; i < decompressed.size(); i += 12)
    {
        if (i + 12 > decompressed.size())
            break;

        float distance, angle;
        memcpy(&distance, &decompressed[i], sizeof(float));
        memcpy(&angle, &decompressed[i + 4], sizeof(float));

        bool valid = distance != 100000.0;  // TODO fix magic number
        points.emplace_back(distance, angle, valid);
    }
    return points;
}

bool M2M2Lidar::_sendJsonRequest(const std::string& command, const nlohmann::json& args)
{
    nlohmann::json request = {
        {"command", command},
        {"request_id", _requestId++}};

    if (!args.is_null())
    {
        request["args"] = args;
    }

    std::string json_str = request.dump();
    std::vector<uint8_t> data(json_str.begin(), json_str.end());

    // append the delimiters
    for (int i = 0; i < 3; ++i)
    {
        data.push_back('\r');
        data.push_back('\n');
    }

    ssize_t sent = send(_socket, data.data(), data.size(), 0);
    if (sent <= 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Send failed: %s", strerror(errno));
        return false;
    }
    return true;
}

nlohmann::json M2M2Lidar::_receiveJsonResponse()
{
    std::vector<uint8_t> buffer(4096);
    std::vector<uint8_t> received;
    bool found_delim = false;
    ssize_t bytes = 0;

    while (!found_delim)
    {
        ssize_t bytes = recv(_socket, buffer.data(), buffer.size(), 0);
        if (bytes <= 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Receive error: %s", strerror(errno));
            return {};
        }
    }

    received.insert(received.end(), buffer.begin(), buffer.begin() + bytes);

    if (received.size() >= 4 &&
        received[received.size() - 4] == '\r' &&
        received[received.size() - 3] == '\n' &&
        received[received.size() - 2] == '\r' &&
        received[received.size() - 1] == '\n')
    {
        found_delim = true;
    }

    std::string response_str(received.begin(), received.end());
    try
    {
        return nlohmann::json::parse(response_str);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON response: %s", e.what());
        return {};
    }
}

/**
 * @brief Creates a configuration command packet for the M2M2 LIDAR sensor.
 *
 * @details Constructs a byte sequence following the sensor's protocol format for
 * configuration commands. Note that this function may become deprecated as hardware
 * testing progresses - the m2m2 LIDAR's default configuration may be sufficient
 * without manual configuration. Testing will confirm or not.
 *
 */
std::vector<uint8_t> M2M2Lidar::_createConfigCommand(const SensorConfig& config)
{
    // Protocol-related constants
    static constexpr uint8_t PROTOCOL_HEADER = 0xAA;
    static constexpr uint8_t PROTOCOL_FOOTER = 0x55;

    std::vector<uint8_t> command;
    command.push_back(PROTOCOL_HEADER);

    // Add command type (example: 0x01 for configuration)
    command.push_back(0x01);

    // Add configuration parameters if needed - this might go-away post hardware testing
    // Note: This is a simplified example - you'll need to match your actual protocol
    command.push_back(static_cast<uint8_t>(config.scanFrequency));
    command.push_back(static_cast<uint8_t>(config.angularResolution * 100));

    // Add footer
    command.push_back(PROTOCOL_FOOTER);

    return command;
}

void M2M2Lidar::_sendCommand(const std::vector<uint8_t>& command)
{
    ssize_t bytesSent = send(_socket, command.data(), command.size(), 0);
    if (bytesSent != static_cast<ssize_t>(command.size()))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to send complete command");
        // TODO: Throw exception
    }
}

void M2M2Lidar::_readSensorData()
{
    if (!_sendJsonRequest("getlaserscan"))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to send getlaserscan command");
        return;
    }

    auto response = _receiveJsonResponse();
    if (response.empty() ||
        response["result"].is_null() ||
        !response["result"].contains("laser_points"))

    {
        RCLCPP_ERROR(this->get_logger(), "Failed to receive valid laser scan response");
        return;
    }

    std::string base64_points = response["result"]["laser_points"];
    auto points = _decodeLaserPoints(base64_points);

    // populate the LaserScan message
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = this->now();
    scan.header.frame_id = this->get_parameter("frame_id").as_string();

    if (!points.empty())
    {
        scan.angle_min = std::get<0>(points.front());
        scan.angle_max = std::get<0>(points.back());
        scan.angle_increment = (scan.angle_max - scan.angle_min) / (points.size() - 1);
        scan.time_increment = 0.0;
        scan.scan_time = 0.0;  // TODO: set this based on sensor specs
        scan.range_min = 0.0;  // TODO: set this based on sensor specs
        scan.range_max = 40.0;

        scan.ranges.reserve(points.size());
        for (const auto& [angle, distance, valid] : points)
        {
            scan.ranges.push_back(valid ? distance : 0.0);
        }
    }

    _scanPublisher->publish(scan);

    //*******************************************************

    // Buffer for receiving data
    std::vector<uint8_t> buffer(1024);  // Adjust size based on your sensor's needs
    ssize_t bytesRead = recv(_socket, buffer.data(), buffer.size(), 0);

    if (bytesRead <= 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error reading from sensor");
        return;
    }

    // Resize buffer to actual data received
    buffer.resize(bytesRead);

    // Note: These are placeholder implementations - we need to implement
    // proper parsing based on m2m2 protocol
    // TODO: Error handling
    /*
    auto maybeScan = _parseLaserScanData(buffer);
    if (maybeScan)
    {
        maybeScan->header.stamp = this->now();
        maybeScan->header.frame_id =
            this->get_parameter("frame_id").as_string();
        _scanPublisher->publish(*maybeScan);
    }

    auto maybeImu = _parseImuData(buffer);
    if (maybeImu)
    {
        maybeImu->header.stamp = this->now();
        maybeImu->header.frame_id =
            this->get_parameter("frame_id").as_string();
        _imuPublisher->publish(*maybeImu);
    }
    */

    /*
    *replaced now
    std::optional<sensor_msgs::msg::LaserScan> M2M2Lidar::_parseLaserScanData([[maybe_unused]] const std::vector<uint8_t>& data)
    {
        sensor_msgs::msg::LaserScan scan{};

        // TODO: Implement

        return scan;
    }

    std::optional<sensor_msgs::msg::Imu> M2M2Lidar::_parseImuData([[maybe_unused]] const std::vector<uint8_t>& data)
    {
        sensor_msgs::msg::Imu imu{};

        // TODO: Implement

        return imu;
    }

    */
}


class M2M2LidarNode : public rclcpp::Node {
public:
    M2M2LidarNode() : Node("m2m2_lidar_node") {
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("m2m2_lidar/scan", 10);
        timer_ = this->create_wall_timer(
            100ms, std::bind(&M2M2LidarNode::publish_scan, this));
    }

private:
    void publish_scan() {
        auto message = sensor_msgs::msg::LaserScan();
        // Fill in the LaserScan message
        publisher_->publish(message);
    }

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<M2M2Lidar>();
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        // TODO: Maybe get a logger with rclcpp::get_logger() and log error, but that should be handled internally to the node anyway
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}