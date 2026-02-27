#include "network_recovery/network_monitor.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include <regex>
#include <sstream>

namespace network_recovery
{

    NetworkMonitor::NetworkMonitor(const Config& cfg, rclcpp::Logger logger)
        : cfg_(cfg),
          logger_(logger)
    {
        if (cfg_.wifi_interface.empty())
        {
            cfg_.wifi_interface = detect_wifi_interface();
        }
        if (cfg_.ping_target.empty())
        {
            cfg_.ping_target = detect_default_gateway();
        }
        RCLCPP_INFO(logger_, "NetworkMonitor: interface=%s, ping_target=%s",
                    cfg_.wifi_interface.c_str(), cfg_.ping_target.c_str());
    }

    NetworkMonitor::~NetworkMonitor() { stop(); }

    void NetworkMonitor::start()
    {
        running_ = true;
        thread_ = std::thread(&NetworkMonitor::run, this);
    }

    void NetworkMonitor::stop()
    {
        running_ = false;
        if (thread_.joinable())
        {
            thread_.join();
        }
    }

    // -- Thread-safe accessors --

    bool NetworkMonitor::connected() const
    {
        std::lock_guard<std::mutex> lk(mtx_);
        return connected_;
    }

    int NetworkMonitor::rssi_dbm() const
    {
        std::lock_guard<std::mutex> lk(mtx_);
        return rssi_dbm_;
    }

    float NetworkMonitor::ping_latency_ms() const
    {
        std::lock_guard<std::mutex> lk(mtx_);
        return ping_latency_ms_;
    }

    double NetworkMonitor::last_success_monotonic() const
    {
        std::lock_guard<std::mutex> lk(mtx_);
        return last_success_;
    }

    SignalTrend NetworkMonitor::signal_trend() const
    {
        std::lock_guard<std::mutex> lk(mtx_);
        return signal_trend_;
    }

    float NetworkMonitor::signal_trend_rate() const
    {
        std::lock_guard<std::mutex> lk(mtx_);
        return signal_trend_rate_;
    }

    // -- Auto-detection --

    std::string NetworkMonitor::detect_default_gateway()
    {
        std::ifstream f("/proc/net/route");
        if (!f.is_open())
            return "8.8.8.8";

        std::string line;
        std::getline(f, line);  // skip header
        while (std::getline(f, line))
        {
            std::istringstream iss(line);
            std::string iface, dest, gateway;
            if (!(iss >> iface >> dest >> gateway))
                continue;
            if (dest == "00000000")
            {
                // Gateway is hex, little-endian 32-bit
                unsigned long gw = std::stoul(gateway, nullptr, 16);
                auto b0 = static_cast<uint8_t>(gw & 0xFF);
                auto b1 = static_cast<uint8_t>((gw >> 8) & 0xFF);
                auto b2 = static_cast<uint8_t>((gw >> 16) & 0xFF);
                auto b3 = static_cast<uint8_t>((gw >> 24) & 0xFF);
                char buf[32];
                std::snprintf(buf, sizeof(buf), "%u.%u.%u.%u", b0, b1, b2, b3);
                return buf;
            }
        }
        return "8.8.8.8";
    }

    std::string NetworkMonitor::detect_wifi_interface()
    {
        std::ifstream f("/proc/net/wireless");
        if (!f.is_open())
            return "";

        std::string line;
        // Skip first two header lines
        std::getline(f, line);
        std::getline(f, line);
        while (std::getline(f, line))
        {
            auto colon = line.find(':');
            if (colon != std::string::npos)
            {
                auto iface = line.substr(0, colon);
                // Trim whitespace
                auto start = iface.find_first_not_of(" \t");
                if (start != std::string::npos)
                {
                    return iface.substr(start);
                }
            }
        }
        return "";
    }

    // -- Monitoring loop --

    void NetworkMonitor::run()
    {
        using namespace std::chrono;
        while (running_)
        {
            double latency = do_ping();
            int rssi = read_rssi();

            {
                std::lock_guard<std::mutex> lk(mtx_);
                if (latency >= 0.0)
                {
                    connected_ = true;
                    ping_latency_ms_ = static_cast<float>(latency);
                    last_success_ =
                        duration<double>(steady_clock::now().time_since_epoch()).count();
                }
                else
                {
                    connected_ = false;
                    ping_latency_ms_ = 0.0f;
                }

                rssi_dbm_ = rssi;

                if (rssi != 0)
                {
                    rssi_window_.push_back(rssi);
                    while (static_cast<int>(rssi_window_.size()) > cfg_.trend_window_size)
                    {
                        rssi_window_.pop_front();
                    }
                }

                compute_trend(signal_trend_, signal_trend_rate_);
            }

            std::this_thread::sleep_for(
                duration<double>(cfg_.ping_interval_s));
        }
    }

    double NetworkMonitor::do_ping()
    {
        if (cfg_.ping_target.empty())
            return -1.0;

        int timeout_s = std::max(1, static_cast<int>(cfg_.ping_timeout_s));
        char cmd[256];
        std::snprintf(cmd, sizeof(cmd), "ping -c 1 -W %d %s 2>/dev/null",
                      timeout_s, cfg_.ping_target.c_str());

        FILE* pipe = popen(cmd, "r");
        if (!pipe)
            return -1.0;

        char buf[512];
        std::string output;
        while (fgets(buf, sizeof(buf), pipe))
        {
            output += buf;
        }
        int status = pclose(pipe);
        if (status != 0)
            return -1.0;

        // Parse "time=12.3 ms" or "time<1 ms"
        std::regex re("time[=<]([\\d.]+)\\s*ms");
        std::smatch match;
        if (std::regex_search(output, match, re))
        {
            return std::stod(match[1].str());
        }
        return 0.1;  // Ping succeeded but couldn't parse time
    }

    int NetworkMonitor::read_rssi()
    {
        if (cfg_.wifi_interface.empty())
            return 0;

        std::ifstream f("/proc/net/wireless");
        if (!f.is_open())
            return 0;

        std::string line;
        while (std::getline(f, line))
        {
            if (line.find(cfg_.wifi_interface) != std::string::npos)
            {
                std::istringstream iss(line);
                std::string iface_field;
                double status_val, link_val, level_val;
                if (!(iss >> iface_field >> status_val >> link_val >> level_val))
                    return 0;
                int level = static_cast<int>(level_val);
                if (level > 0)
                    level -= 256;
                return level;
            }
        }
        return 0;
    }

    void NetworkMonitor::compute_trend(SignalTrend& trend, float& rate) const
    {
        size_t n = rssi_window_.size();
        if (n < 3)
        {
            trend = TREND_STABLE;
            rate = 0.0f;
            return;
        }

        // Linear regression: slope of RSSI over sample indices
        double sum_x = 0.0, sum_x2 = 0.0, sum_y = 0.0, sum_xy = 0.0;
        for (size_t i = 0; i < n; ++i)
        {
            double x = static_cast<double>(i);
            double y = static_cast<double>(rssi_window_[i]);
            sum_x += x;
            sum_x2 += x * x;
            sum_y += y;
            sum_xy += x * y;
        }

        double dn = static_cast<double>(n);
        double denom = dn * sum_x2 - sum_x * sum_x;
        if (std::abs(denom) < 1e-9)
        {
            trend = TREND_STABLE;
            rate = 0.0f;
            return;
        }

        double slope = (dn * sum_xy - sum_x * sum_y) / denom;
        rate = static_cast<float>(slope);

        if (slope < -cfg_.trend_threshold)
        {
            trend = TREND_DEGRADING;
        }
        else if (slope > cfg_.trend_threshold)
        {
            trend = TREND_IMPROVING;
        }
        else
        {
            trend = TREND_STABLE;
        }
    }

}  // namespace network_recovery
