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

#include "m2m2_lidar.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <openssl/bio.h>  // for Base64 decoding
#include <openssl/evp.h>  // also for Base64 decoding
#include <sys/socket.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

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
    if (::connect(_socket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0)
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
        .maxRange = 30.0,
    };
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
    if (_socket >= 0)
    {
        close(_socket);
        _socket = -1;  // defensive - invalidate after cleanup and prevent double close
    }
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
        strncmp(reinterpret_cast<const char*>(decoded.data()), "RLE", 3) != 0)
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
            if (pos + 2 < decoded.size() &&
                decoded[pos + 1] == 0 &&
                decoded[pos + 2] == sentinel2)
            {
                std::swap(sentinel1, sentinel2);
                pos += 2;
            }
            else if (pos + 2 < decoded.size())
            {
                uint8_t repeat_val = decoded[pos + 2];
                uint8_t repeat_count = decoded[pos + 1];
                for (uint8_t i = 0; i < repeat_count; i++)
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

    // Parse points (12 bytes each: float distance, float angle, bool valid)
    for (size_t i = 0; i < decompressed.size(); i += 12)
    {
        if (i + 12 > decompressed.size())
            break;

        float distance, angle;
        std::copy_n(decompressed.data() + i, sizeof(float), reinterpret_cast<uint8_t*>(&distance));
        std::copy_n(decompressed.data() + i + 4, sizeof(float), reinterpret_cast<uint8_t*>(&angle));

        bool valid = distance != 100000.0;

        if (valid)
        {
            RCLCPP_DEBUG(this->get_logger(), "Point: angle=%.2f, distance=%.2f",
                         angle, distance);
        }

        points.emplace_back(angle, distance, valid);
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
    RCLCPP_DEBUG(this->get_logger(), "Sending JSON request: %s", json_str.c_str());

    std::vector<uint8_t> data(json_str.begin(), json_str.end());

    // append the delimiters
    for (int i = 0; i < 3; ++i)
    {
        data.push_back('\r');
        data.push_back('\n');
    }

    RCLCPP_DEBUG(this->get_logger(), "Sending %zu bytes to socket", data.size());

    ssize_t sent = send(_socket, data.data(), data.size(), 0);
    if (sent <= 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Send failed: %s (errno: %d)", strerror(errno), errno);
        return false;
    }

    RCLCPP_DEBUG(this->get_logger(), "Successfully sent %zd bytes", sent);
    return true;
}

nlohmann::json M2M2Lidar::_receiveJsonResponse()
{
    std::vector<uint8_t> buffer(4096);
    std::vector<uint8_t> received;
    bool found_delim = false;
    auto start_time = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::seconds(2);

    RCLCPP_DEBUG(this->get_logger(), "Waiting for response...");

    while (!found_delim)
    {
        // Check for timeout
        auto now = std::chrono::steady_clock::now();
        if (now - start_time > timeout)
        {
            RCLCPP_ERROR(this->get_logger(), "Timeout waiting for response");
            return nlohmann::json();
        }

        ssize_t bytes = recv(_socket, buffer.data(), buffer.size(), MSG_DONTWAIT);
        if (bytes < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                // No data available yet, sleep briefly and try again
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            RCLCPP_ERROR(this->get_logger(), "Receive error: %s (errno: %d)", strerror(errno), errno);
            return nlohmann::json();
        }
        else if (bytes == 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Connection closed by peer");
            return nlohmann::json();
        }

        RCLCPP_DEBUG(this->get_logger(), "Received %zd bytes", bytes);
        received.insert(received.end(), buffer.begin(), buffer.begin() + bytes);

        // Print first few bytes for debugging
        std::stringstream ss;
        for (size_t i = 0; i < static_cast<size_t>(std::min(bytes, ssize_t(16))); ++i)
        {
            ss << std::hex << std::setw(2) << std::setfill('0')
               << static_cast<int>(buffer[i]) << " ";
        }
        RCLCPP_DEBUG(this->get_logger(), "First bytes: %s", ss.str().c_str());

        // Check for delimiter
        if (received.size() >= 4 &&
            received[received.size() - 4] == '\r' &&
            received[received.size() - 3] == '\n' &&
            received[received.size() - 2] == '\r' &&
            received[received.size() - 1] == '\n')
        {
            found_delim = true;
            RCLCPP_DEBUG(this->get_logger(), "Found delimiter after %zu bytes", received.size());
        }
    }

    // Remove the delimiter
    received.resize(received.size() - 4);

    std::string response_str(received.begin(), received.end());
    try
    {
        RCLCPP_DEBUG(this->get_logger(), "Parsing JSON response of length %zu", response_str.length());
        auto json = nlohmann::json::parse(response_str);
        RCLCPP_DEBUG(this->get_logger(), "Successfully parsed JSON");
        return json;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "JSON parse error: %s\nFirst 100 bytes of response: %s",
                     e.what(), response_str.substr(0, 100).c_str());
        return nlohmann::json();
    }
}

bool M2M2Lidar::connectToSensor(const std::string& host, int port)
{
    RCLCPP_INFO(this->get_logger(), "Connecting to %s:%d", host.c_str(), port);

    _socket = socket(AF_INET, SOCK_STREAM, 0);
    if (_socket < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Socket creation failed: %s (errno: %d)",
                     strerror(errno), errno);
        return false;
    }

    // Set socket timeout
    struct timeval tv;
    tv.tv_sec = 2;  // 2 second timeout
    tv.tv_usec = 0;
    if (setsockopt(_socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to set receive timeout: %s", strerror(errno));
        close(_socket);
        return false;
    }
    if (setsockopt(_socket, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to set send timeout: %s", strerror(errno));
        close(_socket);
        return false;
    }

    // Enable TCP keepalive
    int keepalive = 1;
    if (setsockopt(_socket, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive)) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to set keepalive: %s", strerror(errno));
        close(_socket);
        return false;
    }

    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);

    if (inet_pton(AF_INET, host.c_str(), &serv_addr.sin_addr) <= 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid address: %s (errno: %d)",
                     strerror(errno), errno);
        close(_socket);
        return false;
    }

    if (!this->connectToSensor(_sensorAddress, _sensorPort))
    {
        RCLCPP_ERROR(this->get_logger(), "Connection failed");
        close(_socket);
        throw std::runtime_error("Failed to connect to LIDAR");
    }

    // Test the connection with a simple request
    if (!_sendJsonRequest("ping"))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to send initial ping");
        close(_socket);
        return false;
    }

    auto response = _receiveJsonResponse();
    if (response.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "No response to initial ping");
        close(_socket);
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Successfully connected to %s:%d", host.c_str(), port);
    return true;
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
    RCLCPP_DEBUG(this->get_logger(), "Requesting laser scan data...");

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
    RCLCPP_DEBUG(this->get_logger(), "Received base64 data of length: %zu", base64_points.length());

    auto points = _decodeLaserPoints(base64_points);
    RCLCPP_DEBUG(this->get_logger(), "Decoded %zu points", points.size());

    if (points.empty())
    {
        RCLCPP_WARN(this->get_logger(), "No valid points decoded from laser scan");
        return;
    }

    // Populate the LaserScan message
    auto scan = std::make_unique<sensor_msgs::msg::LaserScan>();
    scan->header.stamp = this->now();
    scan->header.frame_id = this->get_parameter("frame_id").as_string();

    scan->angle_min = std::get<0>(points.front());
    scan->angle_max = std::get<0>(points.back());
    scan->angle_increment = (scan->angle_max - scan->angle_min) / (points.size() - 1);
    scan->time_increment = 1.0 / (15.0 * points.size());  // Assuming 15Hz scan rate
    scan->scan_time = 1.0 / 15.0;                         // 15Hz
    scan->range_min = 0.1;                                // 10cm minimum range
    scan->range_max = 30.0;                               // 30m maximum range

    scan->ranges.reserve(points.size());
    for (const auto& [angle, distance, valid] : points)
    {
        scan->ranges.push_back(valid ? distance : 0.0);
    }

    RCLCPP_DEBUG(this->get_logger(), "Publishing scan with %zu ranges", scan->ranges.size());
    _scanPublisher->publish(*scan);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<M2M2Lidar>();
        RCLCPP_INFO(rclcpp::get_logger("main"), "Starting M2M2Lidar node...");
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Error running M2M2Lidar: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
