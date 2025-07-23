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

#include <algorithm>
#include <bit>  // For std::bit_cast
#include <cassert>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <numbers>
#include <ranges>
#include <stdexcept>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "simple_networking/client.hpp"  //supports the nampespace usage in the constructor

const std::string M2M2Lidar::IMU_COMMAND = "getimuinrobotcoordinate";

using std::string;
using std::vector;
using namespace std::chrono_literals;

M2M2Lidar::M2M2Lidar(const rclcpp::NodeOptions& options)
    : Node("m2m2_lidar", options)
{
    using namespace networking;

    // Set up ROS logging to DEBUG explicitly
    // auto ret = rcutils_logging_set_logger_level(
    //     get_logger().get_name(),
    //     RCUTILS_LOG_SEVERITY_DEBUG);  // Changed from INFO to DEBUG

    // if (ret != RCUTILS_RET_OK)
    // {
    //     RCLCPP_ERROR(this->get_logger(), "Failed to set logger level to DEBUG");
    // }
    // else
    // {
    //     RCLCPP_DEBUG(this->get_logger(), "Logger level set to DEBUG successfully");
    // }

    // Load and validate parameters
    RCLCPP_DEBUG(this->get_logger(), "Starting M2M2Lidar initialization...");
    this->declare_parameter("sensor_ip", "192.168.1.243");
    this->declare_parameter("sensor_port", 1445);
    this->declare_parameter("frame_id", "laser_frame");
    this->declare_parameter("scan_topic", "scan");
    this->declare_parameter("imu_topic", "imu");
    this->declare_parameter("imu_frame_id", "imu_frame");
    this->declare_parameter("imu_rate", 100);   // Hz
    this->declare_parameter("read_imu", true);  // if imu data should be read

    // Get the parameters
    std::string sensorIp = this->get_parameter("sensor_ip").as_string();
    uint16_t sensorPort = this->get_parameter("sensor_port").as_int();

    RCLCPP_INFO(this->get_logger(),
                "Attempting connection to sensor at %s:%d, read_imu: %s",
                sensorIp.c_str(), sensorPort, _readImu ? "true" : "false");

    // Configure socket handlers
    socket_config_handlers_t handlers{
        .preBind = nullptr,
        .preConnect = [this](int fd) -> bool
        {
            RCLCPP_DEBUG(this->get_logger(), "Setting up socket options...");

            struct timeval timeoutValue{
                .tv_sec = 5,
                .tv_usec = 0};

            if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeoutValue, sizeof(timeoutValue)) < 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to set receive timeout: %s", strerror(errno));
                return false;
            }

            if (setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeoutValue, sizeof(timeoutValue)) < 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to set send timeout: %s", strerror(errno));
                return false;
            }

            int keepaliveEnabled = 1;
            if (setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &keepaliveEnabled, sizeof(keepaliveEnabled)) < 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to set keepalive: %s", strerror(errno));
                return false;
            }

            RCLCPP_DEBUG(this->get_logger(), "Socket options configured successfully");
            return true;
        },
        .postConnect = nullptr};

    RCLCPP_INFO(this->get_logger(), "Creating network client...");

    try
    {
        // Initialize the client
        _client.emplace(
            address_t{
                .hostname = sensorIp,
                .service = std::to_string(sensorPort)},
            socket_protocol::TCP,
            address_t{},
            handlers);
        RCLCPP_INFO(this->get_logger(), "Network client created successfully");
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to create network client: %s", e.what());
        throw;
    }

    // Perform initial test but don’t throw on failure
    RCLCPP_DEBUG(this->get_logger(), "Starting initial getlaserscan test...");
    nlohmann::json request = {
        {"command", "getlaserscan"},
        {"request_id", _requestId++}};
    std::string full_message = request.dump() + "\r\n\r\n";
    RCLCPP_DEBUG(this->get_logger(), "Sending test request: %s", full_message.c_str());

    if (_sendJsonRequest("getlaserscan"))
    {
        auto testResponse = _receiveJsonResponse();
        if (testResponse.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No response received from sensor during init test, proceeding anyway");
        }
        else if (!testResponse.contains("result") || !testResponse["result"].contains("code") || testResponse["result"]["code"] != 1)
        {
            RCLCPP_WARN(this->get_logger(), "Invalid response during init test: %s, proceeding anyway",
                        testResponse.dump().c_str());
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(), "Connection test successful, configuring sensor...");
        }
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Failed to send initial getlaserscan command, proceeding anyway");
    }

    // Send initial configuration regardless of test outcome
    sensor_config_t defaultConfig{
        .scanFrequency = 15.0,
        .angularResolution = 0.25,
        .minRange = 0.1,
        .maxRange = 30.0,
    };
    RCLCPP_DEBUG(this->get_logger(), "Sending initial configuration...");
    _sendCommand(_createConfigCommand(defaultConfig));

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Initialize publishers and timers
    RCLCPP_DEBUG(this->get_logger(), "Initializing publishers...");

    _initializePublishers();

    RCLCPP_DEBUG(this->get_logger(), "Creating timers...");
    _readTimer = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&M2M2Lidar::_readSensorData, this));

    if (_readImu)
    {
        const auto initial_imu_period = std::chrono::milliseconds(50);
        _imuTimer = this->create_wall_timer(
            initial_imu_period,
            [this]()
            {
                static int call_count = 0;
                _readImuData();
                if (call_count++ == 10)
                {
                    RCLCPP_DEBUG(this->get_logger(), "Switching IMU timer to full speed (100Hz)");
                    _imuTimer->cancel();
                    _imuTimer = this->create_wall_timer(
                        std::chrono::milliseconds(10),
                        std::bind(&M2M2Lidar::_readImuData, this));
                }
            });
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "IMU reading disabled by parameter 'read_imu'");
    }

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

    // RCLCPP_INFO(this->get_logger(), "Preparing to send command: %s", command.c_str());

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
    RCLCPP_DEBUG(this->get_logger(), "Full message to send: %s", full_message.c_str());

    // Use the networking library to send the data
    try
    {
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
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Exception during transmit: %s", e.what());
        return false;
    }
}

nlohmann::json M2M2Lidar::_receiveJsonResponse()
{
    assert(_client && "Network client should be initialised by constructor");

    std::vector<uint8_t> received;
    const auto timeout = std::chrono::seconds(5);
    auto startTime = std::chrono::steady_clock::now();
    bool foundDelim = false;

    RCLCPP_DEBUG(this->get_logger(), "Waiting for response...");

    while (!foundDelim)
    {
        auto now = std::chrono::steady_clock::now();
        if (now - startTime > timeout)
        {
            if (received.empty())
            {
                RCLCPP_INFO(this->get_logger(), "No data received from sensor after %ld seconds from initialisation",
                            std::chrono::duration_cast<std::chrono::seconds>(timeout).count());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Timeout waiting for response delimiter. Received %zu bytes: %.100s",
                             received.size(), std::string(received.begin(), received.end()).c_str());
                RCLCPP_DEBUG(this->get_logger(), "Full received data on timeout: %s",
                             std::string(received.begin(), received.end()).c_str());
            }
            return nlohmann::json();
        }

        try
        {
            RCLCPP_DEBUG(this->get_logger(), "Attempting to receive data...");
            auto chunk = _client->receive(4096, false);
            if (!chunk)
            {
                RCLCPP_DEBUG(this->get_logger(), "No data available yet in this iteration");
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            else
            {
                RCLCPP_DEBUG(this->get_logger(), "Received chunk of %zu bytes: %s",
                             chunk->size(),
                             std::string(chunk->begin(), chunk->end()).c_str());
            }

            received.insert(received.end(), chunk->begin(), chunk->end());
            RCLCPP_DEBUG(this->get_logger(), "Current buffer size: %zu bytes, content: %s",
                         received.size(),
                         std::string(received.begin(), received.end()).c_str());
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception during receive: %s", e.what());
            RCLCPP_DEBUG(this->get_logger(), "Buffer state on exception: %s",
                         std::string(received.begin(), received.end()).c_str());
            return nlohmann::json();
        }

        if (received.size() >= 4)
        {
            std::string_view lastFour(
                reinterpret_cast<const char*>(&received[received.size() - 4]),
                4);
            if (lastFour == "\r\n\r\n")
            {
                foundDelim = true;
                RCLCPP_DEBUG(this->get_logger(), "Found delimiter after %zu bytes", received.size());
                RCLCPP_DEBUG(this->get_logger(), "Complete message received: %s",
                             std::string(received.begin(), received.end() - 4).c_str());
            }
        }
    }

    std::string json_str(received.begin(), received.end() - 4);
    try
    {
        auto json_response = nlohmann::json::parse(json_str);
        RCLCPP_DEBUG(this->get_logger(), "Parsed JSON: %s", json_response.dump().c_str());
        return json_response;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON response: %s", e.what());
        RCLCPP_DEBUG(this->get_logger(), "Raw data that failed parsing: %s", json_str.c_str());
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
        !response["result"].contains("laser_points") ||
        response["result"]["code"] != 1)  // 1 is success code
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to receive valid laser scan response");
        return;
    }

    string base64Points = response["result"]["laser_points"];
    auto rawPoints = _decodeLaserPoints(base64Points);
    if (rawPoints.empty())
    {
        RCLCPP_WARN(this->get_logger(), "No valid points decoded from laser scan");
        return;
    }

    auto points = _interpolatePoints(rawPoints);

    // Populate the LaserScan message
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = this->now();

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
    static int error_count = 0;

    RCLCPP_DEBUG(this->get_logger(), "Requesting IMU data...");

    if (!_sendJsonRequest(IMU_COMMAND))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to send IMU data request");
        error_count++;
        if (error_count > 5)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Multiple IMU request failures - may need to reconnect");
        }

        return;
    }

    auto response = _receiveJsonResponse();
    if (response.empty() || !response.contains("result"))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to receive valid IMU response");
        error_count++;
        if (error_count > 5)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Multiple IMU response failures - may need to reconnect");
        }
        return;
    }

    error_count = 0;

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

std::vector<std::tuple<float, float, bool>> M2M2Lidar::_interpolatePoints(
    const std::vector<std::tuple<float, float, bool>>& input_points)
{
    using namespace std;

    if (input_points.empty())
        return {};

    // First normalize and sort points by angle
    vector<tuple<float, float, bool>> points;
    points.reserve(input_points.size());

    // Normalize angles to [0, 2*pi] range
    for (const auto& point : input_points)
    {
        float angle = get<0>(point);
        /**
         * @brief Normalize angle to [0, 2π] range
         *
         * @details This normalization is crucial for consistent LIDAR point processing:
         *          - Ensures all angles are positive and less than 2*pi
         *          - Handles wrap-around cases (e.g. -pi/2 becomes 3*pi / 2)
         *
         */
        const auto tau = 2 * std::numbers::pi;
        angle -= std::trunc(angle / tau) * tau;
        if (angle < 0)
            angle += tau;
        points.emplace_back(angle, get<1>(point), get<2>(point));
    }

    // Sort by angle
    sort(points.begin(), points.end(),
         [](const auto& a, const auto& b)
         { return get<0>(a) < get<0>(b); });

    vector<tuple<float, float, bool>> interpolated;
    interpolated.reserve(INTERPOLATED_POINTS);

    // Calculate angle increment
    float angleIncrement = 2 * std::numbers::pi / INTERPOLATED_POINTS;

    // Track index between iterations since points are sorted by angle
    size_t sourcePointIndex = 0;

    // For each desired interpolation point
    for (size_t i = 0; i < INTERPOLATED_POINTS; ++i)
    {
        float targetAngle = i * angleIncrement;

        // Find points that bracket our target angle
        // C++20 approach using ranges to find next point
        // 1. views::drop - Skip elements we've already processed
        // 2. find_if - Search for first point with angle >= target
        // 3. distance - Get index of found point

        if (auto foundIterator = std::ranges::find_if(points | std::views::drop(sourcePointIndex),
                                                      [targetAngle](const auto& point)
                                                      {
                                                          return std::get<0>(point) >= targetAngle;
                                                      });
            foundIterator != points.end())
        {
            sourcePointIndex = std::distance(points.begin(), foundIterator);
        }
        else
        {
            sourcePointIndex = points.size();
        }

        // Handle edge cases
        if (sourcePointIndex == 0)
        {
            // Interpolate between last and first point
            auto p1 = points.back();
            auto p2 = points.front();

            // Adjust angle for wrap-around
            float a1 = get<0>(p1);
            float a2 = get<0>(p2) + 2 * std::numbers::pi;

            if (!get<2>(p1) && !get<2>(p2))
            {
                interpolated.emplace_back(targetAngle, INVALID_DISTANCE, false);
                continue;
            }

            if (!get<2>(p1))
            {
                interpolated.emplace_back(targetAngle, get<1>(p2), true);
                continue;
            }

            if (!get<2>(p2))
            {
                interpolated.emplace_back(targetAngle, get<1>(p1), true);
                continue;
            }

            float t = (targetAngle - a1) / (a2 - a1);
            float dist = std::lerp(get<1>(p1), get<1>(p2), t);

            interpolated.emplace_back(targetAngle, dist, true);
        }
        else if (sourcePointIndex == points.size())
        {
            // Use last point's value
            interpolated.emplace_back(targetAngle, get<1>(points.back()), get<2>(points.back()));
        }
        else
        {
            // Normal interpolation between two points
            auto& p1 = points[sourcePointIndex - 1];
            auto& p2 = points[sourcePointIndex];

            if (!get<2>(p1) && !get<2>(p2))
            {
                interpolated.emplace_back(targetAngle, INVALID_DISTANCE, false);
                continue;
            }

            if (!get<2>(p1))
            {
                interpolated.emplace_back(targetAngle, get<1>(p2), true);
                continue;
            }

            if (!get<2>(p2))
            {
                interpolated.emplace_back(targetAngle, get<1>(p1), true);
                continue;
            }

            float t = (targetAngle - get<0>(p1)) / (get<0>(p2) - get<0>(p1));
            float dist = std::lerp(get<1>(p1), get<1>(p2), t);

            interpolated.emplace_back(targetAngle, dist, true);
        }
    }

    return interpolated;
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
