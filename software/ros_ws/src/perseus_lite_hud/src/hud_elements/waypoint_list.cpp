#include "perseus_lite_hud/hud_elements/waypoint_list.hpp"

namespace perseus_lite_hud
{

    WaypointList::WaypointList(const WaypointListConfig& cfg)
        : HudElementBase("waypoint_list"),
          cfg_(cfg)
    {
    }

    void WaypointList::set_waypoints(const std::vector<WaypointEntry>& waypoints,
                                     uint32_t current_index, bool active)
    {
        waypoints_ = waypoints;
        current_index_ = current_index;
        active_ = active;
        has_data_ = true;
    }

    void WaypointList::render(cv::Mat& /*frame*/)
    {
        // TODO: Implement waypoint list rendering
    }

    bool WaypointList::is_ready() const
    {
        return has_data_;
    }

}  // namespace perseus_lite_hud
