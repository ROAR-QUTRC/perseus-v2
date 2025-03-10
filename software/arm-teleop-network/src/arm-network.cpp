#include "arm-network.hpp"

#include <ctime>
#include <iostream>

namespace perseus
{

    // Constructor for server (follower) mode
    ArmNetworkInterface::ArmNetworkInterface(uint16_t port)
        : _mode(Mode::Server), _host(""), _port(port)
    {
    }

    // Constructor for client (leader) mode
    ArmNetworkInterface::ArmNetworkInterface(const std::string& host, uint16_t port)
        : _mode(Mode::Client), _host(host), _port(port)
    {
    }

    // Destructor
    ArmNetworkInterface::~ArmNetworkInterface()
    {
        stop();
    }

    // Start the network interface
    bool ArmNetworkInterface::start()
    {
        if (_running)
        {
            return false;  // Already running
        }

        _running = true;

        // Start the network thread
        _networkThread = std::thread(&ArmNetworkInterface::_networkThreadFunc, this);

        // Start the heartbeat thread
        _heartbeatThread = std::thread(&ArmNetworkInterface::_heartbeatThreadFunc, this);

        return true;
    }

    // Stop the network interface
    void ArmNetworkInterface::stop()
    {
        _running = false;

        if (_networkThread.joinable())
        {
            _networkThread.join();
        }

        if (_heartbeatThread.joinable())
        {
            _heartbeatThread.join();
        }

        _connected = false;
    }

    // Check if connected
    bool ArmNetworkInterface::isConnected() const
    {
        return _connected;
    }

    // Set callbacks for received messages
    void ArmNetworkInterface::setServoPositionsCallback(const ServoPositionsCallback& callback)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _servoPositionsCallback = callback;
    }

    void ArmNetworkInterface::setServoMirroringCallback(const ServoMirroringCallback& callback)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _servoMirroringCallback = callback;
    }

    void ArmNetworkInterface::setCalibrationCallback(const CalibrationCallback& callback)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _calibrationCallback = callback;
    }

    void ArmNetworkInterface::setStatusCallback(const StatusCallback& callback)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _statusCallback = callback;
    }

    // Send messages
    bool ArmNetworkInterface::sendServoPositions(const ServoPositionsMessage& message)
    {
        auto data = serializeMessage(message);
        _queueMessage(data);
        return true;
    }

    bool ArmNetworkInterface::sendServoMirroring(const ServoMirroringMessage& message)
    {
        auto data = serializeMessage(message);
        _queueMessage(data);
        return true;
    }

    bool ArmNetworkInterface::sendCalibration(const CalibrationMessage& message)
    {
        auto data = serializeMessage(message);
        _queueMessage(data);
        return true;
    }

    bool ArmNetworkInterface::sendStatus(const StatusMessage& message)
    {
        auto data = serializeMessage(message);
        _queueMessage(data);
        return true;
    }

    // Network thread function
    void ArmNetworkInterface::_networkThreadFunc()
    {
        std::cout << "Network thread started in "
                  << (_mode == Mode::Client ? "client" : "server")
                  << " mode" << std::endl;

        const auto reconnect_interval = std::chrono::seconds(5);

        while (_running)
        {
            try
            {
                if (_mode == Mode::Client)
                {
                    if (!_client)
                    {
                        std::cout << "Connecting to " << _host << ":" << _port << "..." << std::endl;

                        // Create the client and attempt to connect
                        _client = std::make_unique<networking::Client>(
                            networking::address_t{
                                .hostname = _host,
                                .service = std::to_string(_port)},
                            networking::socket_protocol::TCP);

                        _connected = true;
                        std::cout << "Connected to " << _host << ":" << _port << std::endl;
                    }

                    // Send any queued messages
                    {
                        std::lock_guard<std::mutex> lock(_outgoingMutex);
                        while (!_outgoingMessages.empty() && _connected)
                        {
                            try
                            {
                                auto& message = _outgoingMessages.front();
                                _client->transmit(message);
                                _outgoingMessages.pop();
                            }
                            catch (const std::exception& e)
                            {
                                std::cerr << "Error sending message: " << e.what() << std::endl;
                                _connected = false;
                                break;
                            }
                        }
                    }

                    // Check for incoming messages
                    try
                    {
                        auto data = _client->receive(1024, false);
                        if (data)
                        {
                            _processMessage(*data);
                        }
                    }
                    catch (const std::exception& e)
                    {
                        std::cerr << "Error receiving message: " << e.what() << std::endl;
                        _connected = false;
                    }

                    // If disconnected, clean up and prepare to reconnect
                    if (!_connected && _client)
                    {
                        _client.reset();
                        std::this_thread::sleep_for(reconnect_interval);
                    }
                }
                else
                {
                    // Server mode not implemented in this example
                    // For a complete implementation, you would:
                    // 1. Create a TCP server socket
                    // 2. Accept connections
                    // 3. Handle multiple clients if needed
                    // 4. Process messages similar to client mode

                    // For simplicity, we're showing client mode only for now
                    std::cerr << "Server mode not implemented in this example" << std::endl;
                    _running = false;
                }

                // Brief sleep to prevent CPU spinning
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            catch (const std::exception& e)
            {
                std::cerr << "Network error: " << e.what() << std::endl;
                _connected = false;

                // Clean up and prepare to reconnect
                if (_client)
                {
                    _client.reset();
                }

                std::this_thread::sleep_for(reconnect_interval);
            }
        }

        std::cout << "Network thread stopped" << std::endl;
    }

    // Heartbeat thread function
    void ArmNetworkInterface::_heartbeatThreadFunc()
    {
        std::cout << "Heartbeat thread started" << std::endl;

        const auto heartbeat_interval = std::chrono::seconds(3);

        while (_running)
        {
            if (_connected)
            {
                // Create and send heartbeat message
                HeartbeatMessage heartbeat;
                heartbeat.header.protocol_version = PROTOCOL_VERSION;
                heartbeat.header.type = MessageType::HEARTBEAT;
                heartbeat.header.payload_length = 4;
                heartbeat.timestamp = static_cast<uint32_t>(std::time(nullptr));

                auto data = serializeMessage(heartbeat);
                _queueMessage(data);
            }

            // Wait for the next heartbeat interval
            std::this_thread::sleep_for(heartbeat_interval);
        }

        std::cout << "Heartbeat thread stopped" << std::endl;
    }

    // Process a received message
    void ArmNetworkInterface::_processMessage(const std::vector<uint8_t>& data)
    {
        if (data.size() < 4)
        {
            std::cerr << "Received message too short" << std::endl;
            return;
        }

        // Extract message type from header
        MessageType type = static_cast<MessageType>(data[1]);

        std::lock_guard<std::mutex> lock(_mutex);

        switch (type)
        {
        case MessageType::SERVO_POSITIONS:
            if (_servoPositionsCallback)
            {
                ServoPositionsMessage message;
                if (deserializeMessage(data, message))
                {
                    _servoPositionsCallback(message);
                }
            }
            break;

        case MessageType::SERVO_MIRRORING:
            if (_servoMirroringCallback)
            {
                ServoMirroringMessage message;
                if (deserializeMessage(data, message))
                {
                    _servoMirroringCallback(message);
                }
            }
            break;

        case MessageType::CALIBRATION_DATA:
            if (_calibrationCallback)
            {
                CalibrationMessage message;
                if (deserializeMessage(data, message))
                {
                    _calibrationCallback(message);
                }
            }
            break;

        case MessageType::STATUS_INFO:
            if (_statusCallback)
            {
                StatusMessage message;
                if (deserializeMessage(data, message))
                {
                    _statusCallback(message);
                }
            }
            break;

        case MessageType::HEARTBEAT:
            // Process heartbeat to update connection status
            HeartbeatMessage message;
            if (deserializeMessage(data, message))
            {
                // Heartbeat received, connection is alive
                _connected = true;
            }
            break;

        default:
            std::cerr << "Unknown message type: " << static_cast<int>(type) << std::endl;
            break;
        }
    }

    // Helper to queue a message for sending
    void ArmNetworkInterface::_queueMessage(const std::vector<uint8_t>& message)
    {
        std::lock_guard<std::mutex> lock(_outgoingMutex);
        _outgoingMessages.push(message);
    }

}  // namespace perseus