#pragma once

#include <mutex>
#include <perseus-arm-teleop.hpp>
#include <simple_networking.hpp>
#include <string>
#include <vector>

#include "networked_arm_controller.hpp"

namespace arm_teleop
{
    /**
     * @brief Controller for the follower arm that receives commands from leader
     */
    class FollowerController : public NetworkedArmController
    {
    public:
        /**
         * @brief Constructor
         * @param bind_address Address to bind the server to (empty for any)
         * @param port Port number to listen on
         */
        FollowerController(const std::string& bind_address = "", uint16_t port = protocol::DEFAULT_PORT);

        /**
         * @brief Destructor
         */
        ~FollowerController() override;

        /**
         * @brief Initialize network server
         * @return true if initialization was successful
         */
        bool initialize() override;

        /**
         * @brief Start the network server
         * @return true if started successfully
         */
        bool start() override;

        /**
         * @brief Set the ST3215 servo controller for the follower arm
         * @param servo_controller Pointer to the servo controller
         */
        void setServoController(ST3215ServoReader* servo_controller);

        /**
         * @brief Get the current status message
         * @return Status message string
         */
        std::string getStatusMessage() const override;

        /**
         * @brief Get servo data for the follower arm
         * @return Vector of servo data
         */
        std::vector<ServoData> getServoData() const;

    private:
        // Thread function for network communication
        void _networkThreadFunc() override;

        // Process received message
        bool _processMessage(const std::vector<uint8_t>& message);

        // Handle specific message types
        bool _handlePositionMessage(const protocol::servo_position_msg_t& msg);
        bool _handleCalibrationMessage(const protocol::calibration_msg_t& msg);
        bool _handleMirrorCommand(const protocol::mirror_command_msg_t& msg);
        bool _handleHandshake(const protocol::handshake_msg_t& msg);
        bool _handleKeepalive(const protocol::keepalive_msg_t& msg);

        // Respond to keepalive message
        bool _sendKeepaliveResponse(uint16_t sequence);

        // Network address information
        std::string _bindAddress;
        uint16_t _port;

        // Servo controller
        ST3215ServoReader* _servoController;

        // Server socket implementation
        // Note: simple-networking doesn't have a server class, so we'll use raw sockets
        int _serverSocket;
        int _clientSocket;
        std::mutex _socketLock;

        // Servo data
        std::vector<ServoData> _servoData;
        std::mutex _dataLock;

        // Connection status
        std::atomic<bool> _connected{false};
        std::atomic<int> _missedKeepalives{0};
    };

}  // namespace arm_teleop