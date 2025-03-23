#include "arm-network.hpp"

#include <ctime>
#include <iostream>
#include <boost/asio.hpp>
#include <fstream>

static std::ofstream debug_log("arm_network_debug.log");

//Note Server is implemented in this file and not simple-networking.

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
                else // Server mode
{
    try {
        static bool server_initialized = false;
        static boost::asio::io_service io_service;
        static boost::asio::ip::tcp::acceptor acceptor(io_service);
        static boost::asio::ip::tcp::socket socket(io_service);
        static std::vector<uint8_t> receive_buffer(1024);
        
        // Log timestamp for each message
        auto now = std::chrono::system_clock::now();
        auto time_t_now = std::chrono::system_clock::to_time_t(now);
        debug_log << "[" << std::ctime(&time_t_now) << "] ";
        
        if (!server_initialized) {
            debug_log << "Starting server on port " << _port << "..." << std::endl;
            
            try {
                // Set up the TCP server
                debug_log << "Creating endpoint for port " << _port << std::endl;
                boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::tcp::v4(), _port);
                
                debug_log << "Opening acceptor" << std::endl;
                acceptor.open(endpoint.protocol());
                
                debug_log << "Setting socket options" << std::endl;
                acceptor.set_option(boost::asio::ip::tcp::acceptor::reuse_address(true));
                
                debug_log << "Binding to endpoint" << std::endl;
                acceptor.bind(endpoint);
                
                debug_log << "Starting to listen" << std::endl;
                acceptor.listen();
                
                server_initialized = true;
                debug_log << "Server successfully initialized on port " << _port << ", waiting for connections..." << std::endl;
            }
            catch (const boost::system::system_error& e) {
                debug_log << "Boost error initializing server: " << e.what() << " (error code: " << e.code() << ")" << std::endl;
                throw; // Re-throw to outer catch block
            }
            catch (const std::exception& e) {
                debug_log << "Error initializing server: " << e.what() << std::endl;
                throw; // Re-throw to outer catch block
            }
        }
        
        // Non-blocking accept
        if (!_connected) {
            debug_log << "Checking for new connections..." << std::endl;
            boost::system::error_code ec;
            
            // Start an accept
            acceptor.accept(socket, ec);
            
            if (!ec) {
                // Connection successful
                debug_log << "Client connected from " << socket.remote_endpoint().address().to_string() 
                          << ":" << socket.remote_endpoint().port() << std::endl;
                _connected = true;
            }
            else if (ec != boost::asio::error::would_block && ec != boost::asio::error::try_again) {
                // Log any error that's not just "no connection available"
                debug_log << "Accept error: " << ec.message() << " (code: " << ec << ")" << std::endl;
            }
        }
        
        // Process data for connected client
        if (_connected && socket.is_open()) {
            // Send any queued messages
            {
                std::lock_guard<std::mutex> lock(_outgoingMutex);
                if (!_outgoingMessages.empty()) {
                    debug_log << "Sending " << _outgoingMessages.size() << " queued messages" << std::endl;
                }
                
                while (!_outgoingMessages.empty()) {
                    try {
                        auto& message = _outgoingMessages.front();
                        boost::asio::write(socket, boost::asio::buffer(message));
                        debug_log << "Sent message of size " << message.size() << std::endl;
                        _outgoingMessages.pop();
                    }
                    catch (const std::exception& e) {
                        debug_log << "Error sending message: " << e.what() << std::endl;
                        _connected = false;
                        socket.close();
                        break;
                    }
                }
            }
            
            // Check for incoming data (non-blocking)
            boost::system::error_code ec;
            size_t available = socket.available(ec);
            
            if (ec) {
                debug_log << "Error checking socket: " << ec.message() << std::endl;
                _connected = false;
                socket.close();
            }
            else if (available > 0) {
                debug_log << "Detected " << available << " bytes available to read" << std::endl;
                
                // Make sure buffer is large enough
                if (receive_buffer.size() < available) {
                    receive_buffer.resize(available);
                    debug_log << "Resized receive buffer to " << available << " bytes" << std::endl;
                }
                
                // Read the data
                size_t bytes_read = socket.read_some(boost::asio::buffer(receive_buffer), ec);
                
                if (!ec && bytes_read > 0) {
                    debug_log << "Read " << bytes_read << " bytes of data" << std::endl;
                    // Create a correctly sized buffer with just the data we received
                    std::vector<uint8_t> data(receive_buffer.begin(), receive_buffer.begin() + bytes_read);
                    _processMessage(data);
                }
                else if (ec) {
                    debug_log << "Error reading from socket: " << ec.message() << std::endl;
                    _connected = false;
                    socket.close();
                }
            }
        }
        
        // Prevent CPU spinning
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    catch (const std::exception& e) {
        debug_log << "Server error: " << e.what() << std::endl;
        _connected = false;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
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