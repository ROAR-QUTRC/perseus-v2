#include <chrono>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

class MapSaver : public rclcpp::Node
{
public:
    MapSaver()
        : Node("map_saver")
    {
        this->declare_parameter<double>("save_interval_sec", 30.0);

        const char* home = std::getenv("HOME");
        std::string default_map_path = std::string(home ? home : "") + "/maps/scan.pcd";
        this->declare_parameter<std::string>("map_file_path", default_map_path);

        save_interval_ = this->get_parameter("save_interval_sec").as_double();
        map_file_path_ = this->get_parameter("map_file_path").as_string();

        client_ = this->create_client<std_srvs::srv::Trigger>("/map_save");

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(save_interval_),
            std::bind(&MapSaver::save_map, this));

        RCLCPP_INFO(this->get_logger(), "Map saver started, saving every %.0fs", save_interval_);
    }

private:
    void save_map()
    {
        if (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Map save service not available");
            return;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = client_->async_send_request(
            request,
            std::bind(&MapSaver::save_callback, this, std::placeholders::_1));
    }

    void save_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
    {
        auto result = future.get();
        if (result->success)
        {
            std::string base = map_file_path_;
            std::string ext = ".pcd";
            auto dot = base.rfind('.');
            if (dot != std::string::npos)
            {
                ext = base.substr(dot);
                base = base.substr(0, dot);
            }

            auto now = std::chrono::system_clock::now();
            std::time_t t = std::chrono::system_clock::to_time_t(now);
            char buf[32];
            std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", std::localtime(&t));

            std::string timestamped = base + "_" + buf + ext;
            try
            {
                std::filesystem::copy_file(
                    map_file_path_, timestamped,
                    std::filesystem::copy_options::overwrite_existing);
                RCLCPP_INFO(this->get_logger(), "Map saved and copied to %s", timestamped.c_str());
            }
            catch (const std::filesystem::filesystem_error& e)
            {
                RCLCPP_WARN(this->get_logger(), "Could not copy map file: %s", e.what());
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Map save failed: %s", result->message.c_str());
        }
    }

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    double save_interval_;
    std::string map_file_path_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapSaver>());
    rclcpp::shutdown();
    return 0;
}