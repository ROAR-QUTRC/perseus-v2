/// @file slip_channel_monitor.cpp
/// @brief Implementation of the per-channel slip assessment.

#include "watchdog/mobility_watchdog/slip_channel_monitor.hpp"

#include <cmath>

namespace watchdog
{
    SlipChannelMonitor::SlipChannelMonitor(config_t config)
        : _config(config)
    {
    }

    void SlipChannelMonitor::update(double commanded, double measured, const rclcpp::Time& now)
    {
        _last_commanded = commanded;
        _last_measured = measured;

        const double commanded_magnitude = std::abs(commanded);
        if (commanded_magnitude < _config.min_commanded_magnitude)
        {
            // Too small to distinguish commanded motion from noise: forget any run in
            // progress rather than let a stale window carry over into the next command.
            _samples.clear();
            _is_command_active = false;
            _is_evaluating = false;
            _is_slipping = false;
            _slip_ratio = 0.0;
            return;
        }

        if (!_is_command_active)
        {
            _is_command_active = true;
            _command_active_since = now;
        }

        const double ratio = 1.0 - (std::abs(measured) / commanded_magnitude);
        _samples.push_back({now, ratio});
        _prune_old_samples(now);

        _is_evaluating = (now - _command_active_since).seconds() >= _config.activation_grace_period_s;
        if (!_is_evaluating)
        {
            _is_slipping = false;
            _slip_ratio = 0.0;
            return;
        }

        double ratio_sum = 0.0;
        for (const auto& sample : _samples)
        {
            ratio_sum += sample.ratio;
        }
        _slip_ratio = ratio_sum / static_cast<double>(_samples.size());
        _is_slipping = _slip_ratio >= _config.slip_ratio_threshold;
    }

    bool SlipChannelMonitor::is_evaluating() const
    {
        return _is_evaluating;
    }

    bool SlipChannelMonitor::is_slipping() const
    {
        return _is_slipping;
    }

    double SlipChannelMonitor::slip_ratio() const
    {
        return _slip_ratio;
    }

    double SlipChannelMonitor::commanded_value() const
    {
        return _last_commanded;
    }

    double SlipChannelMonitor::measured_value() const
    {
        return _last_measured;
    }

    void SlipChannelMonitor::_prune_old_samples(const rclcpp::Time& now)
    {
        while (!_samples.empty() && (now - _samples.front().stamp).seconds() > _config.detection_window_s)
        {
            _samples.pop_front();
        }
    }

}  // namespace watchdog
