// arm-network-mini.cpp
#include "arm-network-mini.hpp"

#include <iostream>

namespace perseus
{
    ArmNetworkInterfaceMini::ArmNetworkInterfaceMini(Mode mode, const std::string& host, uint16_t port)
        : _mode(mode),
          _host(host),
          _port(port)
    {
        _socket = std::make_unique<boost::asio::ip::tcp::socket>(_io_context);
    }

    ArmNetworkInterfaceMini::~ArmNetworkInterfaceMini() { stop(); }

    bool ArmNetworkInterfaceMini::start()
    {
        if (_running)
            return false;
        _running = true;
        _networkThread = std::thread(&ArmNetworkInterfaceMini::_networkThreadFunc, this);
        _heartbeatThread = std::thread(&ArmNetworkInterfaceMini::_heartbeatThreadFunc, this);
        return true;
    }

    void ArmNetworkInterfaceMini::stop()
    {
        _running = false;
        if (_socket && _socket->is_open())
            _socket->close();
        if (_networkThread.joinable())
            _networkThread.join();
        if (_heartbeatThread.joinable())
            _heartbeatThread.join();
        _connected = false;
    }

    bool ArmNetworkInterfaceMini::isConnected() const { return _connected; }

    void ArmNetworkInterfaceMini::setServoCallback(const ServoCallback& callback)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _servoCallback = callback;
    }

    bool ArmNetworkInterfaceMini::sendServoPositions(const ServoPositionsMessage& message)
    {
        auto data = serializeMessage(message);
        _queueMessage(data);
        return true;
    }

    void ArmNetworkInterfaceMini::_networkThreadFunc()
    {
        using namespace boost::asio;
        ip::tcp::acceptor acceptor(_io_context);
        ip::tcp::endpoint endpoint(ip::tcp::v4(), _port);

        if (_mode == Mode::Server)
        {
            acceptor.open(endpoint.protocol());
            acceptor.bind(endpoint);
            acceptor.listen();
            acceptor.accept(*_socket);
            _connected = true;
        }
        else
        {
            ip::tcp::resolver resolver(_io_context);
            auto endpoints = resolver.resolve(_host, std::to_string(_port));
            connect(*_socket, endpoints);
            _connected = true;
        }

        while (_running)
        {
            std::lock_guard<std::mutex> lock(_outgoingMutex);
            while (!_outgoingMessages.empty() && _connected)
            {
                auto& msg = _outgoingMessages.front();
                write(*_socket, buffer(msg));
                _outgoingMessages.pop();
            }

            if (_socket->available())
            {
                std::vector<uint8_t> data(_socket->available());
                _socket->read_some(buffer(data));
                _processMessage(data);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void ArmNetworkInterfaceMini::_heartbeatThreadFunc()
    {
        while (_running)
        {
            if (_connected)
            {
                HeartbeatMessage hb{{PROTOCOL_VERSION, MessageType::HEARTBEAT, 4},
                                    static_cast<uint32_t>(std::time(nullptr))};
                _queueMessage(serializeMessage(hb));
            }
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }
    }

    void ArmNetworkInterfaceMini::_processMessage(const std::vector<uint8_t>& data)
    {
        if (data.size() < 4)
            return;
        MessageType type = static_cast<MessageType>(data[1]);
        std::lock_guard<std::mutex> lock(_mutex);
        if (type == MessageType::SERVO_POSITIONS && _servoCallback)
        {
            ServoPositionsMessage msg;
            if (deserializeMessage(data, msg))
                _servoCallback(msg);
        }
        else if (type == MessageType::HEARTBEAT)
        {
            _connected = true;
        }
    }

    void ArmNetworkInterfaceMini::_queueMessage(const std::vector<uint8_t>& message)
    {
        std::lock_guard<std::mutex> lock(_outgoingMutex);
        _outgoingMessages.push(message);
    }
}