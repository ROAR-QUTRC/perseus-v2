// arm-network-mini.hpp
#pragma once
#include <atomic>
#include <boost/asio.hpp>
#include <mutex>
#include <queue>
#include <thread>

#include "arm-servo-protocol-mini.hpp"

namespace perseus
{
    class ArmNetworkInterfaceMini
    {
    public:
        enum class Mode
        {
            Client,
            Server
        };
        using ServoCallback = std::function<void(const ServoPositionsMessage&)>;

        ArmNetworkInterfaceMini(Mode mode, const std::string& host = "", uint16_t port = DEFAULT_PORT);
        ~ArmNetworkInterfaceMini();
        bool start();
        void stop();
        bool isConnected() const;
        void setServoCallback(const ServoCallback& callback);
        bool sendServoPositions(const ServoPositionsMessage& message);

    private:
        Mode _mode;
        std::string _host;
        uint16_t _port;
        std::atomic<bool> _connected{false};
        std::atomic<bool> _running{false};
        std::thread _networkThread;
        std::thread _heartbeatThread;
        ServoCallback _servoCallback;
        std::mutex _mutex;
        boost::asio::io_context _io_context;
        std::unique_ptr<boost::asio::ip::tcp::socket> _socket;
        std::queue<std::vector<uint8_t>> _outgoingMessages;
        std::mutex _outgoingMutex;

        void _networkThreadFunc();
        void _heartbeatThreadFunc();
        void _processMessage(const std::vector<uint8_t>& data);
        void _queueMessage(const std::vector<uint8_t>& message);
    };
}