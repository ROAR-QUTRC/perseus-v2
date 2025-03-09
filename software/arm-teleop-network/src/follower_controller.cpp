#include "arm_teleop/follower_controller.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <format>
#include <iostream>
#include <thread>

// Function to scale a position value from one range to another
static uint16_t scalePosition(uint16_t pos, uint16_t srcMin, uint16_t srcMax, uint16_t destMin, uint16_t destMax)
{
    // Ensure we don't divide by zero
    if (srcMax == srcMin)
        return destMin;

    // Calculate the scaling factor and apply it
    double scale = static_cast<double>(destMax - destMin) / static_cast<double>(srcMax - srcMin);
    return static_cast<uint16_t>(destMin + (pos - srcMin) * scale);
}

namespace arm_teleop
{

    FollowerController::FollowerController(const std::string& bind_address, uint16_t port)
        : NetworkedArmController(), _bindAddress(bind_address), _port(port), _servoController(nullptr), _serverSocket(-1), _clientSocket(-1), _servoData(protocol::NUM_SERVOS), _connected(false), _missedKeepalives(0)
    {
        _statusMessage = std::format("Follower configured to listen on {}:{}",
                                     bind_address.empty() ? "0.0.0.0" : bind_address, port);
    }

    FollowerController::~FollowerController()
    {
        stop();

        // Close any open sockets
        if (_clientSocket >= 0)
        {
            close(_clientSocket);
            _clientSocket = -1;
        }

        if (_serverSocket >= 0)
        {
            close(_serverSocket);
            _serverSocket = -1;
        }
    }

    bool FollowerController::initialize()
    {
        // Create socket
        _serverSocket = socket(AF_INET, SOCK_STREAM, 0);
        if (_serverSocket < 0)
        {
            _statusMessage = std::format("Failed to create socket: {}", strerror(errno));
            return false;
        }

        // Set socket options
        int opt = 1;
        if (setsockopt(_serverSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0)
        {
            _statusMessage = std::format("Failed to set socket options: {}", strerror(errno));
            close(_serverSocket);
            _serverSocket = -1;
            return false;
        }

        // Prepare address structure
        struct sockaddr_in address;
        address.sin_family = AF_INET;

        // Bind to specific address or any address
        if (_bindAddress.empty())
        {
            address.sin_addr.s_addr = INADDR_ANY;
        }
        else
        {
            if (inet_pton(AF_INET, _bindAddress.c_str(), &address.sin_addr) <= 0)
            {
                _statusMessage = std::format("Invalid address: {}", _bindAddress);
                close(_serverSocket);
                _serverSocket = -1;
                return false;
            }
        }

        address.sin_port = htons(_port);

        // Bind the socket
        if (bind(_serverSocket, (struct sockaddr*)&address, sizeof(address)) < 0)
        {
            _statusMessage = std::format("Failed to bind socket: {}", strerror(errno));
            close(_serverSocket);
            _serverSocket = -1;
            return false;
        }

        // Listen for connections
        if (listen(_serverSocket, 1) < 0)
        {
            _statusMessage = std::format("Failed to listen on socket: {}", strerror(errno));
            close(_serverSocket);
            _serverSocket = -1;
            return false;
        }

        _statusMessage = std::format("Server initialized, listening on {}:{}",
                                     _bindAddress.empty() ? "0.0.0.0" : _bindAddress, _port);
        return true;
    }

    bool FollowerController::start()
    {
        if (_running.load())
        {
            _statusMessage = "Follower controller already running";
            return false;
        }

        if (_serverSocket < 0)
        {
            _statusMessage = "Server socket not initialized";
            return false;
        }

        _running.store(true);
        _networkThread = std::make_unique<std::thread>(&FollowerController::_networkThreadFunc, this);
        _statusMessage = "Follower controller started";
        return true;
    }

    void FollowerController::setServoController(ST3215ServoReader* servo_controller)
    {
        _servoController = servo_controller;
    }

    std::string FollowerController::getStatusMessage() const
    {
        return _statusMessage;
    }

    std::vector<NetworkedArmController::ServoData> FollowerController::getServoData() const
    {
        // Create a copy of the data to return
        // We need to use a const_cast here since we're in a const method but need to lock the mutex
        // This is safe because we're not modifying the data, just protecting access to it
        std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(_dataLock));
        return _servoData;
    }

    void FollowerController::_networkThreadFunc()
    {
        constexpr int MAX_MISSED_KEEPALIVES = 10;
        constexpr auto KEEPALIVE_TIMEOUT = std::chrono::seconds(5);

        auto last_received_time = std::chrono::steady_clock::now();
        std::vector<uint8_t> receive_buffer(1024);

        while (_running.load())
        {
            // Accept client connection if not connected
            if (_clientSocket < 0)
            {
                struct sockaddr_in client_addr;
                socklen_t client_len = sizeof(client_addr);

                // Set the server socket to non-blocking mode for the accept call
                int flags = fcntl(_serverSocket, F_GETFL, 0);
                fcntl(_serverSocket, F_SETFL, flags | O_NONBLOCK);

                _clientSocket = accept(_serverSocket, (struct sockaddr*)&client_addr, &client_len);

                // Restore blocking mode
                fcntl(_serverSocket, F_SETFL, flags);

                if (_clientSocket >= 0)
                {
                    // Get client IP address
                    char client_ip[INET_ADDRSTRLEN];
                    inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, sizeof(client_ip));

                    _statusMessage = std::format("Client connected from {}", client_ip);
                    _connected.store(true);
                    last_received_time = std::chrono::steady_clock::now();
                    _missedKeepalives = 0;
                }
                else if (errno != EAGAIN && errno != EWOULDBLOCK)
                {
                    _statusMessage = std::format("Accept failed: {}", strerror(errno));
                }
            }
            else
            {
                // Check for incoming data
                fd_set readfds;
                FD_ZERO(&readfds);
                FD_SET(_clientSocket, &readfds);

                struct timeval tv;
                tv.tv_sec = 0;
                tv.tv_usec = 10000;  // 10ms timeout

                int activity = select(_clientSocket + 1, &readfds, NULL, NULL, &tv);

                if (activity < 0 && errno != EINTR)
                {
                    _statusMessage = std::format("Select error: {}", strerror(errno));
                    close(_clientSocket);
                    _clientSocket = -1;
                    _connected.store(false);
                }
                else if (activity > 0 && FD_ISSET(_clientSocket, &readfds))
                {
                    // Data available
                    ssize_t read_size = read(_clientSocket, receive_buffer.data(), receive_buffer.size());

                    if (read_size > 0)
                    {
                        // Process the received data
                        receive_buffer.resize(read_size);
                        if (_processMessage(receive_buffer))
                        {
                            last_received_time = std::chrono::steady_clock::now();
                            _missedKeepalives = 0;
                        }
                        receive_buffer.resize(1024);  // Reset buffer size for next read
                    }
                    else if (read_size == 0)
                    {
                        // Connection closed by client
                        _statusMessage = "Client disconnected";
                        close(_clientSocket);
                        _clientSocket = -1;
                        _connected.store(false);
                    }
                    else
                    {
                        // Read error
                        _statusMessage = std::format("Read error: {}", strerror(errno));
                        close(_clientSocket);
                        _clientSocket = -1;
                        _connected.store(false);
                    }
                }

                // Check for timeout
                auto current_time = std::chrono::steady_clock::now();
                if (current_time - last_received_time > KEEPALIVE_TIMEOUT)
                {
                    _missedKeepalives++;
                    if (_missedKeepalives >= MAX_MISSED_KEEPALIVES)
                    {
                        _statusMessage = "Connection timeout";
                        close(_clientSocket);
                        _clientSocket = -1;
                        _connected.store(false);
                    }
                }
            }

            // Sleep to avoid consuming too much CPU
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    bool FollowerController::_processMessage(const std::vector<uint8_t>& message)
    {
        if (message.size() < 4)
        {
            _statusMessage = "Received invalid message (too short)";
            return false;
        }

        // Try to identify message type
        protocol::message_type_t msg_type = protocol::identifyMessageType(message.data(), message.size());

        try
        {
            switch (msg_type)
            {
            case protocol::message_type_t::SERVO_POSITIONS:
                if (message.size() == sizeof(protocol::servo_position_msg_t))
                {
                    auto pos_msg = protocol::deserializeMessage<protocol::servo_position_msg_t>(message);
                    return _handlePositionMessage(pos_msg);
                }
                break;

            case protocol::message_type_t::CALIBRATION:
                if (message.size() == sizeof(protocol::calibration_msg_t))
                {
                    auto cal_msg = protocol::deserializeMessage<protocol::calibration_msg_t>(message);
                    return _handleCalibrationMessage(cal_msg);
                }
                break;

            case protocol::message_type_t::MIRROR_COMMAND:
                if (message.size() == sizeof(protocol::mirror_command_msg_t))
                {
                    auto mir_msg = protocol::deserializeMessage<protocol::mirror_command_msg_t>(message);
                    return _handleMirrorCommand(mir_msg);
                }
                break;

            case protocol::message_type_t::HANDSHAKE:
                if (message.size() == sizeof(protocol::handshake_msg_t))
                {
                    auto hs_msg = protocol::deserializeMessage<protocol::handshake_msg_t>(message);
                    return _handleHandshake(hs_msg);
                }
                break;

            case protocol::message_type_t::KEEPALIVE:
                if (message.size() == sizeof(protocol::keepalive_msg_t))
                {
                    auto ka_msg = protocol::deserializeMessage<protocol::keepalive_msg_t>(message);
                    return _handleKeepalive(ka_msg);
                }
                break;

            default:
                _statusMessage = "Received unknown message type";
                return false;
            }

            _statusMessage = "Received invalid message (wrong size)";
            return false;
        }
        catch (const std::exception& e)
        {
            _statusMessage = std::format("Error processing message: {}", e.what());
            return false;
        }
    }

    bool FollowerController::_handlePositionMessage(const protocol::servo_position_msg_t& msg)
    {
        if (!_servoController)
        {
            _statusMessage = "No servo controller set";
            return false;
        }

        {
            std::lock_guard<std::mutex> lock(_dataLock);

            // Update local servo data
            _positionMsgToServoData(msg, _servoData);

            // Apply to physical servos
            for (size_t i = 0; i < protocol::NUM_SERVOS; ++i)
            {
                if (_servoData[i].mirroring)
                {
                    try
                    {
                        // Scale position to servo's min/max range
                        uint16_t scaled_pos = scalePosition(
                            msg.positions[i],
                            msg.positions[i],  // Current position as source range (no scaling)
                            msg.positions[i],
                            _servoData[i].min,
                            _servoData[i].max);

                        // Write position to servo
                        _servoController->writePosition(i + 1, scaled_pos);

                        // Update current position
                        _servoData[i].current = scaled_pos;
                    }
                    catch (const std::exception& e)
                    {
                        _servoData[i].error = e.what();
                        _servoData[i].mirroring = false;
                    }
                }
            }
        }

        return true;
    }

    bool FollowerController::_handleCalibrationMessage(const protocol::calibration_msg_t& msg)
    {
        std::lock_guard<std::mutex> lock(_dataLock);

        // Update local calibration data
        _calibrationMsgToServoData(msg, _servoData);

        _statusMessage = "Received calibration data";
        return true;
    }

    bool FollowerController::_handleMirrorCommand(const protocol::mirror_command_msg_t& msg)
    {
        if (!_servoController)
        {
            _statusMessage = "No servo controller set";
            return false;
        }

        // Validate servo ID
        if (msg.servo_id < 1 || msg.servo_id > protocol::NUM_SERVOS)
        {
            _statusMessage = "Invalid servo ID in mirror command";
            return false;
        }

        const uint8_t servo_index = msg.servo_id - 1;
        const bool enable_mirror = (msg.mirror_enable != 0);

        try
        {
            // Enable/disable torque on the servo
            _servoController->writeControlRegister(msg.servo_id, 0x28, enable_mirror ? 1 : 0);

            // Update mirroring state
            {
                std::lock_guard<std::mutex> lock(_dataLock);
                _servoData[servo_index].mirroring = enable_mirror;
                _servoData[servo_index].error.clear();  // Clear any previous errors
            }

            _statusMessage = std::format("Servo {} mirroring {}", msg.servo_id, enable_mirror ? "enabled" : "disabled");
            return true;
        }
        catch (const std::exception& e)
        {
            _statusMessage = std::format("Failed to set mirroring for servo {}: {}", msg.servo_id, e.what());
            return false;
        }
    }

    bool FollowerController::_handleHandshake(const protocol::handshake_msg_t& msg)
    {
        // Verify the identifier
        if (std::strncmp(msg.identifier, "PERSEUS_ARM_CTRL", sizeof(msg.identifier)) != 0)
        {
            _statusMessage = "Invalid handshake identifier";
            return false;
        }

        // Respond with our own handshake
        try
        {
            protocol::handshake_msg_t response;
            std::strncpy(response.identifier, "PERSEUS_ARM_CTRL", sizeof(response.identifier) - 1);

            auto data = protocol::serializeMessage(response);

            // Send response
            {
                std::lock_guard<std::mutex> lock(_socketLock);
                if (_clientSocket >= 0)
                {
                    ssize_t sent = write(_clientSocket, data.data(), data.size());
                    if (sent != static_cast<ssize_t>(data.size()))
                    {
                        _statusMessage = std::format("Failed to send handshake response: {}", strerror(errno));
                        return false;
                    }
                }
                else
                {
                    return false;
                }
            }

            _statusMessage = "Handshake completed successfully";
            return true;
        }
        catch (const std::exception& e)
        {
            _statusMessage = std::format("Failed to send handshake response: {}", e.what());
            return false;
        }
    }

    bool FollowerController::_handleKeepalive(const protocol::keepalive_msg_t& msg)
    {
        // Send a keepalive response
        return _sendKeepaliveResponse(msg.sequence);
    }

    bool FollowerController::_sendKeepaliveResponse(uint16_t sequence)
    {
        try
        {
            protocol::keepalive_msg_t response;
            response.sequence = sequence;

            auto data = protocol::serializeMessage(response);

            // Send response
            {
                std::lock_guard<std::mutex> lock(_socketLock);
                if (_clientSocket >= 0)
                {
                    ssize_t sent = write(_clientSocket, data.data(), data.size());
                    if (sent != static_cast<ssize_t>(data.size()))
                    {
                        _statusMessage = std::format("Failed to send keepalive response: {}", strerror(errno));
                        return false;
                    }
                }
                else
                {
                    return false;
                }
            }

            return true;
        }
        catch (const std::exception& e)
        {
            _statusMessage = std::format("Failed to send keepalive response: {}", e.what());
            return false;
        }
    }

}  // namespace arm_teleop