#pragma once

#include <mutex>
#include <string>
#include <vector>

#include "../../shared/simple-networking/include/simple_networking.hpp"
#include "networked_arm_controller.hpp"
#include "perseus-arm-teleop.hpp"

namespace arm_teleop
{
    /**
     * @brief Controller for the leader arm that sends commands to follower
     */
    class LeaderController : public NetworkedArmController
    {
    public:
        /**
         * @brief Constructor
         * @param server_address Address of the follower server
         * @param port Port number to connect to
         */
        LeaderController(const std::string& server_address, uint16_t port = protocol::DEFAULT_PORT);

        /**
         * @brief Destructor
         */
        ~LeaderController() override;

        /**
         * @brief Initialize network connection to follower
         * @return true if initialization was successful
         */
        bool initialize() override;

        /**
         * @brief Start the network communication thread
         * @return true if started successfully
         */
        bool start() override;

        /**
         * @brief Update the current servo positions and torque values
         * @param arm1_data Current data for all servos in the leader arm
         */
        void updateServoData(const std::vector<ServoData>& arm1_data);

        /**
         * @brief Send servo mirroring command
         * @param servo_id ID of the servo (1-based)
         * @param enable True to enable mirroring, false to disable
         * @return true if command was sent successfully
         */
        bool sendMirrorCommand(uint8_t servo_id, bool enable);

        /**
         * @brief Send calibration data to follower
         * @param arm1_data Calibration data from leader arm
         * @return true if data was sent successfully
         */
        bool sendCalibrationData(const std::vector<ServoData>& arm1_data);

        /**
         * @brief Get current status message
         * @return Status message string
         */
        std::string getStatusMessage() const override;

    private:
        // Thread function for network communication
        void _networkThreadFunc() override;

        // Send a keepalive message to check if connection is still active
        bool _sendKeepalive();

        // Perform initial handshake with follower
        bool _performHandshake();

        // Connection information
        std::string _serverAddress;
        uint16_t _port;

        // Network client
        std::unique_ptr<networking::Client> _client;

        // Servo data
        std::vector<ServoData> _servoData;
        std::mutex _dataLock;

        // Connection status
        std::atomic<bool> _connected{false};
        std::atomic<uint16_t> _keepaliveSequence{0};
        std::atomic<int> _missedKeepalives{0};
    };

}  // namespace arm_teleop