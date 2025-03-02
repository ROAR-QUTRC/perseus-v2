#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "network_protocol.hpp"

namespace arm_teleop
{
    /**
     * @brief Base class for networked arm controllers
     */
    class NetworkedArmController
    {
    public:
        /**
         * @brief Struct to hold servo data
         */
        struct ServoData
        {
            uint16_t current = 0;
            uint16_t min = 4095;
            uint16_t max = 0;
            int16_t torque = 0;
            std::string error;
            bool mirroring = false;
        };

        /**
         * @brief Constructor
         */
        NetworkedArmController();

        /**
         * @brief Virtual destructor
         */
        virtual ~NetworkedArmController();

        /**
         * @brief Initialize the network controller
         * @return true if initialization was successful
         */
        virtual bool initialize() = 0;

        /**
         * @brief Start the network controller operation
         * @return true if started successfully
         */
        virtual bool start() = 0;

        /**
         * @brief Stop the network controller operation
         */
        virtual void stop();

        /**
         * @brief Check if controller is running
         * @return true if running
         */
        bool isRunning() const
        {
            return _running.load();
        }

        /**
         * @brief Get a status message for display
         * @return Status message string
         */
        virtual std::string getStatusMessage() const = 0;

    protected:
        // Thread function for network operations
        virtual void _networkThreadFunc() = 0;

        // Helper to convert ServoData array to protocol message
        protocol::servo_position_msg_t _servoDataToPositionMsg(const std::vector<ServoData>& servoData);

        // Helper to convert protocol message to ServoData array
        void _positionMsgToServoData(const protocol::servo_position_msg_t& msg, std::vector<ServoData>& servoData);

        // Helper to convert ServoData array to calibration message
        protocol::calibration_msg_t _servoDataToCalibrationMsg(const std::vector<ServoData>& servoData);

        // Helper to convert calibration message to ServoData array
        void _calibrationMsgToServoData(const protocol::calibration_msg_t& msg, std::vector<ServoData>& servoData);

        // Protected members
        std::atomic<bool> _running{false};
        std::unique_ptr<std::thread> _networkThread;
        std::string _statusMessage;
    };

}  // namespace arm_teleop