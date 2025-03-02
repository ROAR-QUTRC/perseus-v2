#include "arm_teleop/leader_controller.hpp"

#include <chrono>
#include <format>
#include <iostream>
#include <thread>

namespace arm_teleop
{

    LeaderController::LeaderController(const std::string& server_address, uint16_t port)
        : NetworkedArmController(), _serverAddress(server_address), _port(port), _client(nullptr), _servoData(protocol::NUM_SERVOS), _connected(false), _keepaliveSequence(0), _missedKeepalives(0)
    {
        _statusMessage = std::format("Leader configured for server: {}:{}", server_address, port);
    }

    LeaderController::~LeaderController()
    {
        stop();
    }

    bool LeaderController::initialize()
    {
        try
        {
            // Create the network client
            _client = std::make_unique<networking::Client>(
                networking::address_t{.hostname = _serverAddress, .service = std::to_string(_port)},
                networking::socket_protocol::TCP);

            _statusMessage = "Successfully connected to server";

            // Perform initial handshake
            if (!_performHandshake())
            {
                _statusMessage = "Handshake failed with server";
                return false;
            }

            _connected.store(true);
            return true;
        }
        catch (const std::exception& e)
        {
            _statusMessage = std::format("Failed to connect to server: {}", e.what());
            return false;
        }
    }

    bool LeaderController::start()
    {
        if (_running.load())
        {
            _statusMessage = "Leader controller already running";
            return false;
        }

        if (!_client)
        {
            _statusMessage = "Client not initialized";
            return false;
        }

        _running.store(true);
        _networkThread = std::make_unique<std::thread>(&LeaderController::_networkThreadFunc, this);
        _statusMessage = "Leader controller started";
        return true;
    }

    void LeaderController::updateServoData(const std::vector<ServoData>& arm1_data)
    {
        if (!_running.load() || !_connected.load())
        {
            return;
        }

        // Lock for thread safety
        std::lock_guard<std::mutex> lock(_dataLock);

        // Store the new servo data
        _servoData = arm1_data;

        // Create and serialize position message
        protocol::servo_position_msg_t msg = _servoDataToPositionMsg(arm1_data);
        auto data = protocol::serializeMessage(msg);

        try
        {
            // Send the data
            _client->transmit(data);
        }
        catch (const std::exception& e)
        {
            _statusMessage = std::format("Failed to send position data: {}", e.what());
            _connected.store(false);
        }
    }

    bool LeaderController::sendMirrorCommand(uint8_t servo_id, bool enable)
    {
        if (!_running.load() || !_connected.load())
        {
            return false;
        }

        // Create the mirror command message
        protocol::mirror_command_msg_t msg;
        msg.servo_id = servo_id;
        msg.mirror_enable = enable ? 1 : 0;

        // Serialize and send
        auto data = protocol::serializeMessage(msg);

        try
        {
            _client->transmit(data);
            return true;
        }
        catch (const std::exception& e)
        {
            _statusMessage = std::format("Failed to send mirror command: {}", e.what());
            _connected.store(false);
            return false;
        }
    }

    bool LeaderController::sendCalibrationData(const std::vector<ServoData>& arm1_data)
    {
        if (!_running.load() || !_connected.load())
        {
            return false;
        }

        // Create the calibration message
        protocol::calibration_msg_t msg = _servoDataToCalibrationMsg(arm1_data);

        // Serialize and send
        auto data = protocol::serializeMessage(msg);

        try
        {
            _client->transmit(data);
            return true;
        }
        catch (const std::exception& e)
        {
            _statusMessage = std::format("Failed to send calibration data: {}", e.what());
            _connected.store(false);
            return false;
        }
    }

    std::string LeaderController::getStatusMessage() const
    {
        return _statusMessage;
    }

    void LeaderController::_networkThreadFunc()
    {
        constexpr int MAX_MISSED_KEEPALIVES = 5;
        constexpr auto KEEPALIVE_INTERVAL = std::chrono::milliseconds(500);

        auto last_keepalive_time = std::chrono::steady_clock::now();

        while (_running.load())
        {
            // Check connection status
            if (!_connected.load())
            {
                // Attempt to reconnect
                try
                {
                    // Re-initialize the client
                    _client = std::make_unique<networking::Client>(
                        networking::address_t{.hostname = _serverAddress, .service = std::to_string(_port)},
                        networking::socket_protocol::TCP);

                    // Perform handshake
                    if (_performHandshake())
                    {
                        _connected.store(true);
                        _statusMessage = "Reconnected to server";
                        _missedKeepalives = 0;
                    }
                    else
                    {
                        _statusMessage = "Handshake failed during reconnection";
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                        continue;
                    }
                }
                catch (const std::exception& e)
                {
                    _statusMessage = std::format("Reconnection failed: {}", e.what());
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    continue;
                }
            }

            // Send keepalive periodically
            auto current_time = std::chrono::steady_clock::now();
            if (current_time - last_keepalive_time > KEEPALIVE_INTERVAL)
            {
                if (!_sendKeepalive())
                {
                    _missedKeepalives++;
                    if (_missedKeepalives >= MAX_MISSED_KEEPALIVES)
                    {
                        _connected.store(false);
                        _statusMessage = "Lost connection to server (missed keepalives)";
                    }
                }
                else
                {
                    _missedKeepalives = 0;
                }

                last_keepalive_time = current_time;
            }

            // Check for responses (non-blocking)
            try
            {
                auto data = _client->receive(1024, false);
                if (data)
                {
                    // Process response if needed
                    // For the leader side, we don't need to process most responses
                    // but we could handle status updates from the follower
                }
            }
            catch (const std::exception& e)
            {
                // Handle error
                _statusMessage = std::format("Error receiving data: {}", e.what());
                _connected.store(false);
            }

            // Sleep to avoid consuming too much CPU
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    bool LeaderController::_sendKeepalive()
    {
        if (!_client || !_connected.load())
        {
            return false;
        }

        try
        {
            // Create keepalive message
            protocol::keepalive_msg_t msg;
            msg.sequence = _keepaliveSequence++;

            // Serialize and send
            auto data = protocol::serializeMessage(msg);
            _client->transmit(data);

            return true;
        }
        catch (const std::exception& e)
        {
            _statusMessage = std::format("Keepalive failed: {}", e.what());
            return false;
        }
    }

    bool LeaderController::_performHandshake()
    {
        if (!_client)
        {
            return false;
        }

        try
        {
            // Create handshake message
            protocol::handshake_msg_t msg;
            const char* identifier = "PERSEUS_ARM_CTRL";
            std::strncpy(msg.identifier, identifier, sizeof(msg.identifier) - 1);

            // Serialize and send
            auto data = protocol::serializeMessage(msg);
            _client->transmit(data);

            // Wait for response
            constexpr auto HANDSHAKE_TIMEOUT = std::chrono::seconds(3);
            auto start_time = std::chrono::steady_clock::now();

            while (std::chrono::steady_clock::now() - start_time < HANDSHAKE_TIMEOUT)
            {
                auto response = _client->receive(sizeof(protocol::handshake_msg_t), false);
                if (response)
                {
                    // Parse the response
                    try
                    {
                        auto response_msg = protocol::deserializeMessage<protocol::handshake_msg_t>(*response);

                        // Verify the identifier
                        if (std::strncmp(response_msg.identifier, "PERSEUS_ARM_CTRL", sizeof(response_msg.identifier)) == 0)
                        {
                            return true;
                        }
                    }
                    catch (const std::exception& e)
                    {
                        _statusMessage = std::format("Invalid handshake response: {}", e.what());
                    }
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            _statusMessage = "Handshake timed out";
            return false;
        }
        catch (const std::exception& e)
        {
            _statusMessage = std::format("Handshake failed: {}", e.what());
            return false;
        }
    }

}  // namespace arm_teleop