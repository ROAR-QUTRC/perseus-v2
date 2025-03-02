#include "arm_teleop/networked_arm_controller.hpp"

#include <algorithm>
#include <stdexcept>

namespace arm_teleop
{

    NetworkedArmController::NetworkedArmController()
        : _running(false), _networkThread(nullptr), _statusMessage("Not initialized")
    {
    }

    NetworkedArmController::~NetworkedArmController()
    {
        stop();
    }

    void NetworkedArmController::stop()
    {
        if (_running.exchange(false))
        {
            if (_networkThread && _networkThread->joinable())
            {
                _networkThread->join();
            }
            _networkThread.reset();
        }
    }

    protocol::servo_position_msg_t NetworkedArmController::_servoDataToPositionMsg(
        const std::vector<ServoData>& servoData)
    {
        protocol::servo_position_msg_t msg;

        // Ensure we have the right number of servos
        const size_t servos_to_process = std::min(servoData.size(),
                                                  static_cast<size_t>(protocol::NUM_SERVOS));

        // Copy position and torque data
        for (size_t i = 0; i < servos_to_process; ++i)
        {
            msg.positions[i] = servoData[i].current;
            msg.torques[i] = servoData[i].torque;
        }

        return msg;
    }

    void NetworkedArmController::_positionMsgToServoData(
        const protocol::servo_position_msg_t& msg,
        std::vector<ServoData>& servoData)
    {
        // Ensure we have enough space in the servo data vector
        if (servoData.size() < protocol::NUM_SERVOS)
        {
            servoData.resize(protocol::NUM_SERVOS);
        }

        // Copy position and torque data
        for (size_t i = 0; i < protocol::NUM_SERVOS; ++i)
        {
            servoData[i].current = msg.positions[i];
            servoData[i].torque = msg.torques[i];
        }
    }

    protocol::calibration_msg_t NetworkedArmController::_servoDataToCalibrationMsg(
        const std::vector<ServoData>& servoData)
    {
        protocol::calibration_msg_t msg;

        // Ensure we have the right number of servos
        const size_t servos_to_process = std::min(servoData.size(),
                                                  static_cast<size_t>(protocol::NUM_SERVOS));

        // Copy min/max data
        for (size_t i = 0; i < servos_to_process; ++i)
        {
            msg.min_positions[i] = servoData[i].min;
            msg.max_positions[i] = servoData[i].max;
        }

        return msg;
    }

    void NetworkedArmController::_calibrationMsgToServoData(
        const protocol::calibration_msg_t& msg,
        std::vector<ServoData>& servoData)
    {
        // Ensure we have enough space in the servo data vector
        if (servoData.size() < protocol::NUM_SERVOS)
        {
            servoData.resize(protocol::NUM_SERVOS);
        }

        // Copy min/max data
        for (size_t i = 0; i < protocol::NUM_SERVOS; ++i)
        {
            servoData[i].min = msg.min_positions[i];
            servoData[i].max = msg.max_positions[i];
        }
    }

}  // namespace arm_teleop