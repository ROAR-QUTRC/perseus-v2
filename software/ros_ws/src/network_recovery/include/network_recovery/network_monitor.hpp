#pragma once

#include <atomic>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/logger.hpp>

namespace network_recovery {

enum SignalTrend : uint8_t {
  TREND_STABLE = 0,
  TREND_IMPROVING = 1,
  TREND_DEGRADING = 2,
};

/// Background thread that monitors network connectivity and WiFi signal
/// strength via ping and /proc/net/wireless.
class NetworkMonitor {
 public:
  struct Config {
    std::string ping_target;
    double ping_interval_s{2.0};
    double ping_timeout_s{1.5};
    std::string wifi_interface;
    int trend_window_size{10};
    double trend_threshold{1.0};
  };

  explicit NetworkMonitor(const Config& cfg, rclcpp::Logger logger);
  ~NetworkMonitor();

  void start();
  void stop();

  // Thread-safe accessors
  bool connected() const;
  int rssi_dbm() const;
  float ping_latency_ms() const;
  double last_success_monotonic() const;
  SignalTrend signal_trend() const;
  float signal_trend_rate() const;

 private:
  static std::string detect_default_gateway();
  static std::string detect_wifi_interface();

  void run();
  /// Returns latency in ms, or -1 on failure.
  double do_ping();
  /// Returns RSSI in dBm, or 0 if unavailable.
  int read_rssi();
  void compute_trend(SignalTrend& trend, float& rate) const;

  Config cfg_;
  rclcpp::Logger logger_;

  mutable std::mutex mtx_;
  bool connected_{false};
  int rssi_dbm_{0};
  float ping_latency_ms_{0.0f};
  double last_success_{0.0};
  SignalTrend signal_trend_{TREND_STABLE};
  float signal_trend_rate_{0.0f};

  std::deque<int> rssi_window_;
  std::atomic<bool> running_{false};
  std::thread thread_;
};

}  // namespace network_recovery
