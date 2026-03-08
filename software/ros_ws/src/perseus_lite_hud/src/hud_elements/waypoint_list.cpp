#include "perseus_lite_hud/hud_elements/waypoint_list.hpp"

namespace perseus_lite_hud
{

    WaypointList::WaypointList(const WaypointListConfig& cfg)
        : HudElementBase("waypoint_list"),
          cfg_(cfg)
    {
    }

    void WaypointList::render(cv::Mat& /*frame*/)
    {
        // TODO: Implement waypoint list rendering
    }

    bool WaypointList::is_ready() const
    {
        return false;
    }

}  // namespace perseus_lite_hud
