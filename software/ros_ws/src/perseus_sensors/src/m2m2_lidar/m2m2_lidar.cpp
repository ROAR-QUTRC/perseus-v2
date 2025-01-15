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
#include <netdb.h>  // For getaddrinfo, freeaddrinfo, gai_strerror
#include <netinet/in.h>
#include <openssl/bio.h>  // for Base64 decoding
#include <openssl/evp.h>  // also for Base64 decoding
#include <sys/socket.h>
#include <sys/types.h>  // For general system types
#include <unistd.h>

#include <bit>  // For std::bit_cast

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "simple_networking/client.hpp"  //supports the nampespace usage in the constructor

using std::string;
using std::vector;
using namespace std::chrono_literals;

M2M2Lidar::M2M2Lidar(const rclcpp::NodeOptions& options)
    : Node("m2m2_lidar", options)
{
    using namespace networking;
    // Set up ROS logging
    auto ret = rcutils_logging_set_logger_level(
        get_logger().get_name(),
        RCUTILS_LOG_SEVERITY_DEBUG);

    if (ret != RCUTILS_RET_OK)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to set logger level to DEBUG");
    }

    // Load and validate parameters
    RCLCPP_DEBUG(this->get_logger(), "Starting M2M2Lidar initialization...");
    this->declare_parameter("sensor_ip", "192.168.1.243");
    this->declare_parameter("sensor_port", 1445);
    // this->declare_parameter("frame_id", "lidar_frame");
    this->declare_parameter("frame_id", "laser_frame");  // testing for autonomy mapping
    this->declare_parameter("scan_topic", "scan");
    this->declare_parameter("imu_topic", "imu");
<<<<<<< HEAD
=======
    this->declare_parameter("imu_frame_id", "imu_frame");
    this->declare_parameter("imu_rate", 100);  // Hz
>>>>>>> bd5ccf6 (autonomy: frame update pre m2m2 bugfix)

    // Get the parameters
    std::string sensorIp;
    uint16_t sensorPort;
    sensorIp = this->get_parameter("sensor_ip").as_string();
    sensorPort = this->get_parameter("sensor_port").as_int();

    RCLCPP_INFO(this->get_logger(),
                "Attempting connection to sensor at %s:%d",
                sensorIp.c_str(), sensorPort);

    // Configure socket handlers
    socket_config_handlers_t handlers{
        .preBind = nullptr,
        .preConnect = [this](int fd) -> bool
        {
            struct timeval timeoutValue
            {
                .tv_sec = 2, .tv_usec = 0
            };
            if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeoutValue, sizeof(timeoutValue)) < 0)
            {
                RCLCPP_INFO(this->get_logger(), "Failed to set receive timeout: %s", strerror(errno));
                return false;
            }
            if (setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeoutValue, sizeof(timeoutValue)) < 0)
            {
                RCLCPP_INFO(this->get_logger(), "Failed to set send timeout: %s", strerror(errno));
                return false;
            }
            int keepaliveEnabled = 1;
            if (setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &keepaliveEnabled, sizeof(keepaliveEnabled)) < 0)
            {
                RCLCPP_INFO(this->get_logger(), "Failed to set keepalive: %s", strerror(errno));
                return false;
            }
            return true;
        },
        .postConnect = nullptr};

    // Initialize the client using emplace
    _client.emplace(
        address_t{
            .hostname = sensorIp,
            .service = std::to_string(sensorPort)},
        socket_protocol::TCP,
        address_t{},
        handlers);

    // Test connection with ping
    if (!_sendJsonRequest("ping"))
    {
        throw std::runtime_error("Failed to send initial ping");
    }

    auto pingResponse = _receiveJsonResponse();
    if (pingResponse.empty())
    {
        throw std::runtime_error("No response to initial ping");
    }

    RCLCPP_DEBUG(this->get_logger(), "Ping successful, configuring sensor...");

    // Create and send initial configuration
    sensor_config_t defaultConfig{
        .scanFrequency = 15.0,
        .angularResolution = 0.25,
        .minRange = 0.1,
        .maxRange = 30.0,
    };

    _sendCommand(_createConfigCommand(defaultConfig));

    // Initialise publishers and timer
    _initializePublishers();
    _readTimer = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&M2M2Lidar::_readSensorData, this));

    // Imu timer initialisation
    _imuTimer = this->create_wall_timer(
        std::chrono::milliseconds(10),  // 100Hz rate for IMU data
        std::bind(&M2M2Lidar::_readImuData, this));

    RCLCPP_INFO(this->get_logger(), "M2M2Lidar initialization complete");
}

void M2M2Lidar::_initializePublishers()
{
    const auto qos = rclcpp::QoS(10);
    _scanPublisher = this->create_publisher<sensor_msgs::msg::LaserScan>(
        this->get_parameter("scan_topic").as_string(), qos);
    _imuPublisher = this->create_publisher<sensor_msgs::msg::Imu>(
        this->get_parameter("imu_topic").as_string(), qos);
}

bool M2M2Lidar::isWithinEpsilon(float a, float b, float epsilon)
{
    return std::abs(a - b) <= epsilon;
}

/**
 * @brief Decodes a base64-encoded string into a byte sequence.
 *
 */
vector<uint8_t> M2M2Lidar::_decodeBase64(const string& encoded)
{
    // Calculate the maximum possible decoded length
    size_t maxDecodedLength = ((encoded.length() + 3) / 4) * 3;

    // Create a buffer for the decoded data
    vector<uint8_t> decoded(maxDecodedLength);
    // Create a buffer for the decoded data
    vector<uint8_t> decoded(maxDecodedLength);

    // Create and initialize the decoding context
    EVP_ENCODE_CTX* context = EVP_ENCODE_CTX_new();
    if (!context)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to create Base64 decoding context");
        throw std::runtime_error("Base64 context creation failed");
    }

    try
    {
        EVP_DecodeInit(context);
        try
        {
            EVP_DecodeInit(context);

            // Process the input data
            int outputLength;
            int result = EVP_DecodeUpdate(context,
                                          decoded.data(), &outputLength,
                                          reinterpret_cast<const unsigned char*>(encoded.data()),
                                          encoded.length());

            if (result < 0)
            {
                throw std::runtime_error("Failed to decode Base64 data");
            }

            // Finalize the decoding
            int finalLength;
            result = EVP_DecodeFinal(context, decoded.data() + outputLength, &finalLength);

            if (result < 0)
            {
                throw std::runtime_error("Failed to finalize Base64 decoding");
            }

            // Cleanup and resize
            EVP_ENCODE_CTX_free(context);
            decoded.resize(outputLength + finalLength);

            RCLCPP_DEBUG(this->get_logger(),
                         "Successfully decoded %zu Base64 bytes to %zu bytes",
                         encoded.length(), decoded.size());

            return decoded;
        }
        catch (...)
        {
            EVP_ENCODE_CTX_free(context);
            throw;
        }
    }

    vector<std::tuple<float, float, bool>> M2M2Lidar::_decodeLaserPoints(const string& base64Encoded)
    {
        vector<std::tuple<float, float, bool>> points;

        vector<uint8_t> decoded = _decodeBase64(base64Encoded);

        if (decoded.size() < 9 ||
            strncmp(reinterpret_cast<const char*>(decoded.data()), "RLE", 3) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid RLE header");
            return points;
        }

        // RLE decompression
        vector<uint8_t> decompressed;
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
                    uint8_t repeatValue = decoded[pos + 2];
                    uint8_t repeatCount = decoded[pos + 1];
                    decompressed.insert(decompressed.end(), repeatCount, repeatValue);
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
        static_assert(sizeof(float) == 4, "Expected 32-bit floats");

        for (size_t i = 0; i < decompressed.size(); i += 12)
        {
            if (i + 12 > decompressed.size())
                break;

            // Read bytes into uint32_t first to handle endianness
            uint32_t distanceRaw = 0;
            uint32_t angleRaw = 0;

            // Assuming little-endian data from sensor
            for (int j = 0; j < 4; j++)
            {
                distanceRaw |= (static_cast<uint32_t>(decompressed[i + j]) << (j * 8));
                angleRaw |= (static_cast<uint32_t>(decompressed[i + 4 + j]) << (j * 8));
            }

            // Use std::bit_cast to safely convert between representations
            float distance = std::bit_cast<float>(distanceRaw);
            float angle = std::bit_cast<float>(angleRaw);

            // Validate the values
            if (!std::isfinite(distance) || !std::isfinite(angle))
            {
                RCLCPP_WARN(this->get_logger(),
                            "Received non-finite values: distance=%.2f, angle=%.2f",
                            distance, angle);
                continue;
            }

            bool valid = !isWithinEpsilon(distance, INVALID_DISTANCE);
            // bool valid = distance != 100000.0;

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
        assert(_client && "Network client should be initialised by constructor");

        // Create the JSON request structure
        nlohmann::json request = {
            {"command", command},
            {"request_id", _requestId++}};

        if (!args.is_null())
        {
            request["args"] = args;
        }

        // Convert to string and add delimiters
        std::string full_message = request.dump() + "\r\n\r\n";

        // Use the networking library to send the data
        ssize_t sent = _client->transmit(full_message);

        if (sent != static_cast<ssize_t>(full_message.length()))
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to send complete message: only sent %zd of %zu bytes",
                         sent, full_message.length());
            return false;
        }

        RCLCPP_DEBUG(this->get_logger(), "Successfully sent %zd bytes", sent);
        return true;
    }

    nlohmann::json M2M2Lidar::_receiveJsonResponse()
    {
        assert(_client && "Network client should be initialised by constructor");

        std::vector<uint8_t> received;
        const auto timeout = std::chrono::seconds(2);
        auto startTime = std::chrono::steady_clock::now();
        bool foundDelim = false;

        RCLCPP_DEBUG(this->get_logger(), "Waiting for response...");

        while (!foundDelim)
        {
            // Check for timeout
            auto now = std::chrono::steady_clock::now();
            if (now - startTime > timeout)
            {
                RCLCPP_ERROR(this->get_logger(), "Timeout waiting for response");
                return nlohmann::json();
            }

            // Try to receive some data
            auto chunk = _client->receive(1024, false);  // non-blocking receive

            if (!chunk)
            {
                // No data available yet, sleep briefly and try again
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // Add the received data to our buffer
            received.insert(received.end(), chunk->begin(), chunk->end());

            // Check for delimiter
            if (received.size() >= 4)
            {
                std::string_view lastFour(
                    reinterpret_cast<const char*>(&received[received.size() - 4]),
                    4);
                if (lastFour == "\r\n\r\n")
                {
                    foundDelim = true;
                    RCLCPP_DEBUG(this->get_logger(),
                                 "Found delimiter after %zu bytes", received.size());
                }
            }
        }

        // Remove the delimiter
        received.resize(received.size() - 4);

        // Convert to string and parse JSON
        std::string response_str(received.begin(), received.end());
        try
        {
            auto json = nlohmann::json::parse(response_str);
            RCLCPP_DEBUG(this->get_logger(), "Successfully parsed JSON response");
            return json;
        }
        catch (const nlohmann::json::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "JSON parse error: %s\nResponse preview: %s",
                         e.what(), response_str.substr(0, 100).c_str());
            return nlohmann::json();
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
    vector<uint8_t> M2M2Lidar::_createConfigCommand(const sensor_config_t& config)
    {
        // Protocol-related constants
        static constexpr uint8_t PROTOCOL_HEADER = 0xAA;
        static constexpr uint8_t PROTOCOL_FOOTER = 0x55;

        // Define valid ranges for our parameters
        static constexpr double MIN_SCAN_FREQUENCY = 0.0;
        static constexpr double MAX_SCAN_FREQUENCY = 255.0;  // uint8_t max
        static constexpr double MIN_ANGULAR_RESOLUTION = 0.0;
        static constexpr double MAX_ANGULAR_RESOLUTION = 2.55;

        vector<uint8_t> command;
        command.push_back(PROTOCOL_HEADER);
        command.push_back(0x01);  // command type for configuration

        // clamp scan frequency to valid uint8_t and round to nearest integer
        double clampedFrequency = std::clamp(config.scanFrequency,
                                             MIN_SCAN_FREQUENCY,
                                             MAX_SCAN_FREQUENCY);
        command.push_back(static_cast<uint8_t>(std::round(clampedFrequency)));

        // Clamp angular resolution, scale by 100, and ensure it fits in uint8_t
        double scaledResolution = std::clamp(config.angularResolution * 100.0,
                                             MIN_ANGULAR_RESOLUTION * 100.0,
                                             MAX_ANGULAR_RESOLUTION * 100.0);
        command.push_back(static_cast<uint8_t>(std::round(scaledResolution)));

        // Add configuration parameters if needed - this might go-away post hardware testing
        command.push_back(static_cast<uint8_t>(config.scanFrequency));
        command.push_back(static_cast<uint8_t>(config.angularResolution * 100));

        // Add footer
        command.push_back(PROTOCOL_FOOTER);

        return command;
    }

    void M2M2Lidar::_sendCommand(const std::vector<uint8_t>& command)
    {
        assert(_client && "Network client should be initialised by constructor");

        ssize_t bytesSent = _client->transmit(command);

        if (bytesSent != static_cast<ssize_t>(command.size()))
        {
            throw std::runtime_error(
                std::format("Incomplete command transmission: sent {} of {} bytes",
                            bytesSent, command.size()));
        }

        RCLCPP_DEBUG(this->get_logger(),
                     "Successfully sent command of %zu bytes", command.size());
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

        string base64Points = response["result"]["laser_points"];
        RCLCPP_DEBUG(this->get_logger(), "Received base64 data of length: %zu", base64Points.length());

        auto points = _decodeLaserPoints(base64Points);
        RCLCPP_DEBUG(this->get_logger(), "Decoded %zu points", points.size());

        if (points.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No valid points decoded from laser scan");
            return;
        }

        // Populate the LaserScan message
        sensor_msgs::msg::LaserScan scan;
        scan.header.stamp = this->now();

        // scan->header.frame_id = this->get_parameter("frame_id").as_string();
        scan.header.frame_id = this->get_parameter("frame_id").as_string();

        scan.angle_min = std::get<0>(points.front());
        scan.angle_max = std::get<0>(points.back());
        scan.angle_increment = (scan.angle_max - scan.angle_min) / (points.size() - 1);
        scan.time_increment = 1.0 / (SCAN_FREQUENCY * points.size());
        scan.scan_time = 1.0 / SCAN_FREQUENCY;
        scan.range_min = MIN_RANGE;
        scan.range_max = MAX_RANGE;

        scan.ranges.reserve(points.size());
        for (const auto& [angle, distance, valid] : points)
        {
            scan.ranges.push_back(valid ? distance : 0.0);
        }

        RCLCPP_DEBUG(this->get_logger(), "Publishing scan with %zu ranges", scan.ranges.size());
        _scanPublisher->publish(scan);
    }

    void M2M2Lidar::_readImuData()
    {
        RCLCPP_DEBUG(this->get_logger(), "Requesting IMU data...");

        if (!_sendJsonRequest(IMU_COMMAND))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send IMU data request");
            return;
        }

        auto response = _receiveJsonResponse();
        if (response.empty() || !response.contains("result"))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to receive valid IMU response");
            return;
        }

        // Create and populate IMU message
        auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
        imu_msg->header.stamp = this->now();
        imu_msg->header.frame_id = this->get_parameter("frame_id").as_string();

        // Extract data from response
        const auto& result = response["result"];

        // Set orientation quaternion
        if (result.contains("quaternion"))
        {
            imu_msg->orientation.w = result["quaternion"]["w"];
            imu_msg->orientation.x = result["quaternion"]["x"];
            imu_msg->orientation.y = result["quaternion"]["y"];
            imu_msg->orientation.z = result["quaternion"]["z"];
        }

        // Set angular velocity from gyro
        if (result.contains("gyro"))
        {
            imu_msg->angular_velocity.x = result["gyro"]["x"];
            imu_msg->angular_velocity.y = result["gyro"]["y"];
            imu_msg->angular_velocity.z = result["gyro"]["z"];
        }

        // Set linear acceleration
        if (result.contains("acc"))
        {
            imu_msg->linear_acceleration.x = result["acc"]["x"];
            imu_msg->linear_acceleration.y = result["acc"]["y"];
            imu_msg->linear_acceleration.z = result["acc"]["z"];
        }

        // Set covariance matrices to unknown
        std::fill(imu_msg->orientation_covariance.begin(),
                  imu_msg->orientation_covariance.end(), -1);
        std::fill(imu_msg->angular_velocity_covariance.begin(),
                  imu_msg->angular_velocity_covariance.end(), -1);
        std::fill(imu_msg->linear_acceleration_covariance.begin(),
                  imu_msg->linear_acceleration_covariance.end(), -1);

        _imuPublisher->publish(std::move(imu_msg));
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
<<<<<<< HEAD
=======
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
vector<uint8_t> M2M2Lidar::_createConfigCommand(const sensor_config_t& config)
{
    // Protocol-related constants
    static constexpr uint8_t PROTOCOL_HEADER = 0xAA;
    static constexpr uint8_t PROTOCOL_FOOTER = 0x55;

    // Define valid ranges for our parameters
    static constexpr double MIN_SCAN_FREQUENCY = 0.0;
    static constexpr double MAX_SCAN_FREQUENCY = 255.0;  // uint8_t max
    static constexpr double MIN_ANGULAR_RESOLUTION = 0.0;
    static constexpr double MAX_ANGULAR_RESOLUTION = 2.55;

    vector<uint8_t> command;
    command.push_back(PROTOCOL_HEADER);
    command.push_back(0x01);  // command type for configuration

    // clamp scan frequency to valid uint8_t and round to nearest integer
    double clampedFrequency = std::clamp(config.scanFrequency,
                                         MIN_SCAN_FREQUENCY,
                                         MAX_SCAN_FREQUENCY);
    command.push_back(static_cast<uint8_t>(std::round(clampedFrequency)));

    // Clamp angular resolution, scale by 100, and ensure it fits in uint8_t
    double scaledResolution = std::clamp(config.angularResolution * 100.0,
                                         MIN_ANGULAR_RESOLUTION * 100.0,
                                         MAX_ANGULAR_RESOLUTION * 100.0);
    command.push_back(static_cast<uint8_t>(std::round(scaledResolution)));

    // Add configuration parameters if needed - this might go-away post hardware testing
    command.push_back(static_cast<uint8_t>(config.scanFrequency));
    command.push_back(static_cast<uint8_t>(config.angularResolution * 100));

    // Add footer
    command.push_back(PROTOCOL_FOOTER);

    return command;
}

void M2M2Lidar::_sendCommand(const std::vector<uint8_t>& command)
{
    assert(_client && "Network client should be initialised by constructor");

    ssize_t bytesSent = _client->transmit(command);

    if (bytesSent != static_cast<ssize_t>(command.size()))
    {
        throw std::runtime_error(
            std::format("Incomplete command transmission: sent {} of {} bytes",
                        bytesSent, command.size()));
    }

    RCLCPP_DEBUG(this->get_logger(),
                 "Successfully sent command of %zu bytes", command.size());
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

    string base64Points = response["result"]["laser_points"];
    RCLCPP_DEBUG(this->get_logger(), "Received base64 data of length: %zu", base64Points.length());

    auto points = _decodeLaserPoints(base64Points);
    RCLCPP_DEBUG(this->get_logger(), "Decoded %zu points", points.size());

    if (points.empty())
    {
        RCLCPP_WARN(this->get_logger(), "No valid points decoded from laser scan");
        return;
    }

    // Populate the LaserScan message
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = this->now();

    // scan->header.frame_id = this->get_parameter("frame_id").as_string();
    scan.header.frame_id = this->get_parameter("frame_id").as_string();

    scan.angle_min = std::get<0>(points.front());
    scan.angle_max = std::get<0>(points.back());
    scan.angle_increment = (scan.angle_max - scan.angle_min) / (points.size() - 1);
    scan.time_increment = 1.0 / (SCAN_FREQUENCY * points.size());
    scan.scan_time = 1.0 / SCAN_FREQUENCY;
    scan.range_min = MIN_RANGE;
    scan.range_max = MAX_RANGE;

    scan.ranges.reserve(points.size());
    for (const auto& [angle, distance, valid] : points)
    {
        scan.ranges.push_back(valid ? distance : 0.0);
    }

    RCLCPP_DEBUG(this->get_logger(), "Publishing scan with %zu ranges", scan.ranges.size());
    _scanPublisher->publish(scan);
}

void M2M2Lidar::_readImuData()
{
    RCLCPP_DEBUG(this->get_logger(), "Requesting IMU data...");

    if (!_sendJsonRequest(IMU_COMMAND))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to send IMU data request");
        return;
    }

    auto response = _receiveJsonResponse();
    if (response.empty() || !response.contains("result"))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to receive valid IMU response");
        return;
    }

    // Create and populate IMU message
    auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
    imu_msg->header.stamp = this->now();
    imu_msg->header.frame_id = this->get_parameter("imu_frame_id").as_string();

    // Extract data from response
    const auto& result = response["result"];

    // Set orientation quaternion
    if (result.contains("quaternion"))
    {
        imu_msg->orientation.w = result["quaternion"]["w"];
        imu_msg->orientation.x = result["quaternion"]["x"];
        imu_msg->orientation.y = result["quaternion"]["y"];
        imu_msg->orientation.z = result["quaternion"]["z"];
    }

    // Set angular velocity from gyro
    if (result.contains("gyro"))
    {
        imu_msg->angular_velocity.x = result["gyro"]["x"];
        imu_msg->angular_velocity.y = result["gyro"]["y"];
        imu_msg->angular_velocity.z = result["gyro"]["z"];
    }

    // Set linear acceleration
    if (result.contains("acc"))
    {
        imu_msg->linear_acceleration.x = result["acc"]["x"];
        imu_msg->linear_acceleration.y = result["acc"]["y"];
        imu_msg->linear_acceleration.z = result["acc"]["z"];
    }

    // Set covariance matrices to unknown
    std::fill(imu_msg->orientation_covariance.begin(),
              imu_msg->orientation_covariance.end(), -1);
    std::fill(imu_msg->angular_velocity_covariance.begin(),
              imu_msg->angular_velocity_covariance.end(), -1);
    std::fill(imu_msg->linear_acceleration_covariance.begin(),
              imu_msg->linear_acceleration_covariance.end(), -1);

    _imuPublisher->publish(std::move(imu_msg));
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
>>>>>>> bd5ccf6 (autonomy: frame update pre m2m2 bugfix)
