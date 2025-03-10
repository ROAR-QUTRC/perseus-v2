#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <queue>
#include <simple_networking.hpp>
#include <thread>

#include "arm-servo-protocol.hpp"

namespace perseus
{

    class ArmNetworkInterface
    {
    public:
        // Callback types for received messages
        using ServoPositionsCallback = std::function<void(const ServoPositionsMessage&)>;
        using ServoMirroringCallback = std::function<void(const ServoMirroringMessage&)>;
        using CalibrationCallback = std::function<void(const CalibrationMessage&)>;
        using StatusCallback = std::function<void(const StatusMessage&)>;

        // Constructor for server (follower) mode
        ArmNetworkInterface(uint16_t port = DEFAULT_PORT);

        // Constructor for client (leader) mode
        ArmNetworkInterface(const std::string& host, uint16_t port = DEFAULT_PORT);

        // Destructor
        ~ArmNetworkInterface();

        // Start the network interface
        bool start();

        // Stop the network interface
        void stop();

        // Check if connected
        bool isConnected() const;

        // Set callbacks for received messages
        void setServoPositionsCallback(const ServoPositionsCallback& callback);
        void setServoMirroringCallback(const ServoMirroringCallback& callback);
        void setCalibrationCallback(const CalibrationCallback& callback);
        void setStatusCallback(const StatusCallback& callback);

        // Send messages
        bool sendServoPositions(const ServoPositionsMessage& message);
        bool sendServoMirroring(const ServoMirroringMessage& message);
        bool sendCalibration(const CalibrationMessage& message);
        bool sendStatus(const StatusMessage& message);

    private:
        // Network mode (client or server)
        enum class Mode
        {
            Client,
            Server
        };
        Mode _mode;

        // Network connection details
        std::string _host;
        uint16_t _port;

        // Connection status
        std::atomic<bool> _connected{false};
        std::atomic<bool> _running{false};

        // Network thread
        std::thread _networkThread;
        std::thread _heartbeatThread;

        // Message callbacks
        ServoPositionsCallback _servoPositionsCallback;
        ServoMirroringCallback _servoMirroringCallback;
        CalibrationCallback _calibrationCallback;
        StatusCallback _statusCallback;

        // Mutex for thread safety
        mutable std::mutex _mutex;

        // Client connection (for leader mode)
        std::unique_ptr<networking::Client> _client;

        // Outgoing message queue
        std::queue<std::vector<uint8_t>> _outgoingMessages;
        std::mutex _outgoingMutex;

        // Network thread function
        void _networkThreadFunc();

        // Heartbeat thread function
        void _heartbeatThreadFunc();

        // Process a received message
        void _processMessage(const std::vector<uint8_t>& data);

        // Helper to queue a message for sending
        void _queueMessage(const std::vector<uint8_t>& message);
    };

}  // namespace perseus