#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "sensors/m2m2_lidar/communication.hpp"
#include "sensors/m2m2_lidar/protocol.hpp"

namespace sensors
{
    class M2M2Lidar : public rclcpp::Node
    {
    public:
        /**
         * @brief Constructor for the M2M2Lidar node
         */
        M2M2Lidar()
            : Node("m2m2_lidar")
        {
            // Declare parameters
            this->declare_parameter("sensor_ip", "192.168.1.100");
            this->declare_parameter("sensor_port", 2000);
            this->declare_parameter("frame_id", "lidar_frame");
            this->declare_parameter("scan_topic", "scan");
            this->declare_parameter("imu_topic", "imu");

            // Get parameters
            const auto sensorIp = this->get_parameter("sensor_ip").as_string();
            const auto sensorPort = static_cast<uint16_t>(this->get_parameter("sensor_port").as_int());
            const auto frameId = this->get_parameter("frame_id").as_string();
            const auto scanTopic = this->get_parameter("scan_topic").as_string();
            const auto imuTopic = this->get_parameter("imu_topic").as_string();

            // Initialize communication and protocol handlers
            _communication = std::make_unique<Communication>(sensorIp, sensorPort);
            _protocol = std::make_unique<Protocol>();

            // Create publishers
            _scanPublisher = this->create_publisher<sensor_msgs::msg::LaserScan>(
                scanTopic, rclcpp::QoS(10));
            _imuPublisher = this->create_publisher<sensor_msgs::msg::Imu>(
                imuTopic, rclcpp::QoS(10));

            // Create timer for periodic data reading
            _timer = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&M2M2Lidar::readSensorData, this));

            RCLCPP_INFO(this->get_logger(), "M2M2 Lidar node initialized");
        }

    private:
        /**
         * @brief Timer callback for reading sensor data
         */
        void readSensorData()
        {
            static uint8_t buffer[4096];
            const auto bytesRead = _communication->receiveData(buffer, sizeof(buffer));

            if (bytesRead > 0)
            {
                // Process laser scan data
                auto scanMsg = _protocol->parseLaserScanData(buffer, bytesRead);
                scanMsg.header.stamp = this->now();
                scanMsg.header.frame_id = this->get_parameter("frame_id").as_string();
                _scanPublisher->publish(scanMsg);

                // Process IMU data
                auto imuMsg = _protocol->parseImuData(buffer, bytesRead);
                imuMsg.header.stamp = this->now();
                imuMsg.header.frame_id = this->get_parameter("frame_id").as_string();
                _imuPublisher->publish(imuMsg);
            }
        }

        std::unique_ptr<Communication> _communication;
        std::unique_ptr<Protocol> _protocol;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr _scanPublisher;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _imuPublisher;
        rclcpp::TimerBase::SharedPtr _timer;
    };
} // namespace sensors

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sensors::M2M2Lidar>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
