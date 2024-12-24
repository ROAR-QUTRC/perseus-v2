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

using std::string;
using std::vector;
using namespace std::chrono_literals;

M2M2Lidar::M2M2Lidar(const rclcpp::NodeOptions& options)
    : Node("m2m2_lidar", options)
{
    // First, let's set the logger level to DEBUG to see all messages
    auto ret = rcutils_logging_set_logger_level(
        get_logger().get_name(),
        RCUTILS_LOG_SEVERITY_DEBUG);

    if (ret != RCUTILS_RET_OK)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to set logger level to DEBUG");
    }

    // Parameter setup with detailed logging
    RCLCPP_DEBUG(this->get_logger(), "Starting M2M2Lidar initialization...");
    this->declare_parameter("sensor_ip", "192.168.1.243");
    this->declare_parameter("sensor_port", 1445);
    this->declare_parameter("frame_id", "lidar_frame");
    this->declare_parameter("scan_topic", "scan");
    this->declare_parameter("imu_topic", "imu");

    _sensorAddress = this->get_parameter("sensor_ip").as_string();
    _sensorPort = this->get_parameter("sensor_port").as_int();

    RCLCPP_INFO(this->get_logger(),
                "Attempting connection to sensor at %s:%d",
                _sensorAddress.c_str(), _sensorPort);

    // Setup address hints
    struct addrinfo addressHints, *addressResults, *currentAddress;
    int addressStatus;

    memset(&addressHints, 0, sizeof addressHints);
    addressHints.ai_family = AF_UNSPEC;      // IPv4 or IPv6
    addressHints.ai_socktype = SOCK_STREAM;  // TCP stream sockets

    // char portString[6];
    // snprintf(portString, sizeof(portString), "%d", _sensorPort);
    std::string portString = std::to_string(_sensorPort);

    RCLCPP_DEBUG(this->get_logger(), "Resolving address information...");

    // Get address information with error details
    if ((addressStatus = getaddrinfo(_sensorAddress.c_str(), portString.c_str(), &addressHints, &addressResults)) != 0)
    {
        std::string error_msg = gai_strerror(addressStatus);
        RCLCPP_ERROR(this->get_logger(),
                     "Address resolution failed: %s (Error code: %d)",
                     error_msg.c_str(), addressStatus);
        throw std::runtime_error("Address resolution failed: " + error_msg);
    }

    RCLCPP_DEBUG(this->get_logger(), "Address resolved successfully, attempting connection...");

    // Connection attempt loop with detailed logging
    bool connection_success = false;
    for (currentAddress = addressResults; currentAddress != nullptr; currentAddress = currentAddress->ai_next)
    {
        _socket = socket(currentAddress->ai_family, currentAddress->ai_socktype, currentAddress->ai_protocol);
        if (_socket == -1)
        {
            RCLCPP_DEBUG(this->get_logger(),
                         "Socket creation failed for address family %d: %s",
                         currentAddress->ai_family, strerror(errno));
            continue;
        }

        RCLCPP_DEBUG(this->get_logger(), "Socket created successfully, configuring options...");

        // Socket options configuration with error checking
        struct timeval timeoutValue;
        timeoutValue.tv_sec = 2;
        timeoutValue.tv_usec = 0;

        if (setsockopt(_socket, SOL_SOCKET, SO_RCVTIMEO, &timeoutValue, sizeof(timeoutValue)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to set receive timeout: %s", strerror(errno));
            close(_socket);
            continue;
        }

        if (setsockopt(_socket, SOL_SOCKET, SO_SNDTIMEO, &timeoutValue, sizeof(timeoutValue)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to set send timeout: %s", strerror(errno));
            close(_socket);
            continue;
        }

        int keepaliveEnabled = 1;
        if (setsockopt(_socket, SOL_SOCKET, SO_KEEPALIVE, &keepaliveEnabled, sizeof(keepaliveEnabled)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to set keepalive: %s", strerror(errno));
            close(_socket);
            continue;
        }

        RCLCPP_DEBUG(this->get_logger(), "Socket options configured, attempting connection...");

        // Connection attempt with detailed error reporting
        if (connect(_socket, currentAddress->ai_addr, currentAddress->ai_addrlen) == -1)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Connection attempt failed: %s (errno: %d)",
                         strerror(errno), errno);
            close(_socket);
            _socket = -1;
            continue;
        }

        // If we get here, connection succeeded
        connection_success = true;
        RCLCPP_INFO(this->get_logger(), "Successfully connected to sensor");
        break;
    }

    freeaddrinfo(addressResults);

    if (!connection_success)
    {
        throw std::runtime_error("Failed to connect to LIDAR sensor after trying all available addresses");
    }

    // Test connection with ping
    RCLCPP_DEBUG(this->get_logger(), "Testing connection with ping command...");
    if (!_sendJsonRequest("ping"))
    {
        RCLCPP_ERROR(this->get_logger(), "Ping request failed to send");
        close(_socket);
        throw std::runtime_error("Failed to send initial ping");
    }

    auto pingResponse = _receiveJsonResponse();
    if (pingResponse.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "No response received from ping request");
        close(_socket);
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

    // Initialize publishers and timer
    RCLCPP_DEBUG(this->get_logger(), "Setting up ROS publishers and timers...");
    _initializePublishers();
    _readTimer = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&M2M2Lidar::_readSensorData, this));

    RCLCPP_INFO(this->get_logger(), "M2M2Lidar initialization complete");
}

M2M2Lidar::~M2M2Lidar()
{
    if (_socket >= 0)
    {
        close(_socket);
        _socket = -1;  // defensive - invalidate after cleanup and prevent double close
    }
}

/// Move constructor
M2M2Lidar::M2M2Lidar(M2M2Lidar&& other) noexcept
    : Node(other.get_name())  // Initialize base class
{
    // Use swap for the actual move operation
    swap(*this, other);

    // The other object now has our default-constructed state
    // We should ensure its resources are properly nulled out
    other._socket = -1;
    other._isConnected = false;
    other._scanPublisher.reset();
    other._imuPublisher.reset();
    other._readTimer.reset();
}

// Move assignment operator
M2M2Lidar& M2M2Lidar::operator=(M2M2Lidar&& other) noexcept
{
    if (this != &other)
    {
        // Use swap for the move
        swap(*this, other);

        // Clean up the other object's resources
        if (other._socket >= 0)
        {
            close(other._socket);
            other._socket = -1;
        }
        other._isConnected = false;
        other._scanPublisher.reset();
        other._imuPublisher.reset();
        other._readTimer.reset();
    }
    return *this;
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
    try
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
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Base64 decoding error: %s", e.what());
        return vector<uint8_t>();
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

bool M2M2Lidar::_sendJsonRequest(const string& command, const nlohmann::json& args)
{
    nlohmann::json request = {
        {"command", command},
        {"request_id", _requestId++}};

    if (!args.is_null())
    {
        request["args"] = args;
    }

    string json_str = request.dump();
    RCLCPP_DEBUG(this->get_logger(), "Sending JSON request: %s", json_str.c_str());

    vector<uint8_t> data(json_str.begin(), json_str.end());

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
    vector<uint8_t> buffer(SOCKET_BUFFER_SIZE);
    vector<uint8_t> received;
    bool foundDelim = false;
    auto startTime = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::seconds(2);

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

        if (received.size() >= 4 &&
            strncmp(reinterpret_cast<const char*>(&received[received.size() - 4]), "\r\n\r\n", 4) == 0)
        {
            foundDelim = true;
            RCLCPP_DEBUG(this->get_logger(), "Found delimiter after %zu bytes", received.size());
        }
    }

    // Remove the delimiter
    received.resize(received.size() - 4);

    string response_str(received.begin(), received.end());
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

void M2M2Lidar::_sendCommand(const vector<uint8_t>& command)
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
