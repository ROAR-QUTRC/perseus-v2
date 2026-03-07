#include <csignal>
#include <cstdio>
#include <memory>
#include <string>

#include "perseus_domain_bridge/throttled_bridge.hpp"
#include "rclcpp/rclcpp.hpp"

// Global bridge pointer for signal handler
static std::unique_ptr<domain_bridge_throttled::ThrottledDomainBridge> g_bridge;

static void signal_handler(int /*signum*/)
{
    // Destructor of ThrottledDomainBridge will cancel executors + join threads
    g_bridge.reset();
    std::_Exit(0);
}

int main(int argc, char** argv)
{
    // We manage our own per-domain rclcpp contexts, so we don't call
    // rclcpp::init() globally here. We only need the rclcpp logger.
    rclcpp::Logger logger = rclcpp::get_logger("bridge");

    if (argc < 2)
    {
        RCLCPP_FATAL(
            logger,
            "Usage: bridge <path/to/config.yaml>");
        return 1;
    }

    const std::string config_path = argv[1];
    RCLCPP_INFO(logger, "Loading config: %s", config_path.c_str());

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    try
    {
        g_bridge = std::make_unique<domain_bridge_throttled::ThrottledDomainBridge>(config_path);
        g_bridge->run();
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(logger, "Fatal error: %s", e.what());
        return 1;
    }

    return 0;
}