#pragma once

/// @file global_traversability.hpp
/// @brief Builds a terrain-aware costmap from the 3D lidar map, in place of a global costmap
///        sourced only from a 2D SLAM occupancy grid.

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <grid_map_core/grid_map_core.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <vector>

namespace global_traversability
{
    /// @brief ROS 2 node that turns FAST-LIO's accumulated map cloud into a multi-layer terrain
    ///        grid_map, exported as one nav_msgs/OccupancyGrid per layer plus a combined
    ///        nav2-consumable occupancy costmap.
    ///
    /// A 2D SLAM map only ever records whether a cell is occupied in a single horizontal slice,
    /// which cannot tell a step the robot can climb from a wall it cannot, or a low branch it
    /// would hit from open air at the same (x, y). This node instead analyses the full 3D map
    /// point cloud per cell -- elevation spread, local slope, local roughness, overhead
    /// clearance -- and folds those into an obstacle/inflation cost that nav2's costmap can
    /// consume directly, without needing a 2D SLAM map as an intermediate.
    ///
    /// The input cloud (FAST-LIO's /Laser_map, see autonomy_bringup/config/livox_mid360.yaml's
    /// publish.map_en) is republished in full on every scan and only grows, so recomputing on
    /// every message would be wasted work on a Jetson-class board. Instead the latest message is
    /// cached and the whole pipeline re-runs on a slower timer.
    ///
    /// Deliberately depends on grid_map_core only, not grid_map_ros: see CMakeLists.txt for why,
    /// and _to_occupancy_grid() for the hand-rolled GridMap -> OccupancyGrid conversion that
    /// replaces it.
    class GlobalTraversability : public rclcpp::Node
    {
    public:
        /// @brief Constructs the node, declaring parameters and setting up I/O.
        /// @param options Node options, supplied by main().
        explicit GlobalTraversability(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    private:
        /// @brief Default topic the accumulated 3D map cloud is read from.
        static inline const std::string DEFAULT_POINTCLOUD_TOPIC = "/Laser_map";
        /// @brief Default topic the final occupancy costmap is published on.
        static inline const std::string DEFAULT_COSTMAP_TOPIC = "costmap";
        /// @brief Namespace debug layers are published under, one nav_msgs/OccupancyGrid each.
        static inline const std::string DEFAULT_LAYERS_TOPIC_PREFIX = "layers/";

        /// @brief Default cell size of the generated grid, in metres.
        static constexpr double DEFAULT_RESOLUTION_M = 0.1;
        /// @brief Default margin added around the cloud's bounding box, in metres.
        static constexpr double DEFAULT_MAP_MARGIN_M = 1.0;
        /// @brief Default period between recomputing the costmap from the latest cloud, in seconds.
        static constexpr double DEFAULT_UPDATE_PERIOD_S = 5.0;

        /// @brief Default side length of the square neighbourhood used for the local plane fit
        ///        that steepness, roughness and ridge are derived from, in metres.
        static constexpr double DEFAULT_NEIGHBOURHOOD_RADIUS_M = 0.3;
        /// @brief Default minimum number of valid neighbours required to fit that plane.
        static constexpr int DEFAULT_MIN_PLANE_FIT_POINTS = 4;
        /// @brief Default height a point must clear a cell's ground estimate by before it counts
        ///        toward that cell's overhead clearance, in metres. Filters ground-return noise.
        static constexpr double DEFAULT_GROUND_MARGIN_M = 0.05;
        /// @brief Default minimum point count for a cell to be considered mapped rather than
        ///        border/unknown.
        static constexpr int DEFAULT_MIN_POINTS_PER_CELL = 3;

        /// @brief Default per-cell max-min height spread, above which a cell is an obstacle, in
        ///        metres.
        static constexpr double DEFAULT_MAX_HEIGHT_DIFF_M = 0.15;
        /// @brief Default local slope, above which a cell is an obstacle, in degrees.
        static constexpr double DEFAULT_MAX_SLOPE_DEG = 30.0;
        /// @brief Default local plane-fit residual, above which a cell is an obstacle, in metres.
        static constexpr double DEFAULT_MAX_ROUGHNESS_M = 0.08;
        /// @brief Default overhead clearance the robot needs to pass under a cell, in metres.
        ///        Cells with less are obstacles regardless of ground-level terrain.
        static constexpr double DEFAULT_MIN_CLEARANCE_M = 0.6;
        /// @brief Whether border/unknown cells are treated as obstacles (safe default for a
        ///        vehicle that cannot verify unmapped ground) rather than as free/unknown space.
        static constexpr bool DEFAULT_TREAT_UNKNOWN_AS_OBSTACLE = true;

        /// @brief Default robot radius used as the inscribed (always-lethal) inflation distance,
        ///        in metres.
        static constexpr double DEFAULT_ROBOT_RADIUS_M = 0.3;
        /// @brief Default distance out to which obstacle cost decays, in metres.
        static constexpr double DEFAULT_INFLATION_RADIUS_M = 0.6;
        /// @brief Default exponential decay rate of inflated cost with distance, matching nav2's
        ///        InflationLayer convention.
        static constexpr double DEFAULT_COST_SCALING_FACTOR = 3.0;

        /// @brief Declares every parameter and copies the values into their matching members.
        void _load_parameters();

        /// @brief Caches the latest map cloud; the actual (re)build happens on the update timer.
        /// @param msg Incoming accumulated map cloud.
        void _pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        /// @brief Rebuilds the grid_map and costmap from the most recently cached cloud, if any.
        void _update_costmap();

        /// @brief Bins each point into elevation_min/elevation_max/point_count per cell.
        /// @param cloud Map cloud to accumulate.
        void _accumulate_elevation(const pcl::PointCloud<pcl::PointXYZ>& cloud);

        /// @brief Fills the clearance layer with the lowest overhead point above each cell's
        ///        ground estimate, once elevation_min is known.
        /// @param cloud Map cloud to scan for overhead returns.
        void _compute_clearance(const pcl::PointCloud<pcl::PointXYZ>& cloud);

        /// @brief Fits a local plane around every mapped cell to derive steepness, roughness and
        ///        ridge (split into ridge_bump/ridge_pothole, see the two-topic comment in the
        ///        constructor) from elevation_min.
        void _compute_local_terrain_features();

        /// @brief Marks cells with too few points as border/unknown.
        void _compute_border();

        /// @brief Combines height_diff, steepness, roughness, clearance and border into a binary
        ///        obstacle layer.
        void _compute_obstacle();

        /// @brief Runs a two-pass chamfer distance transform off the obstacle layer and turns the
        ///        result into a decaying inflation cost.
        void _compute_inflation();

        /// @brief Folds obstacle, inflation and border into the final 0-100 (or NaN/unknown)
        ///        cost layer that gets exported as the occupancy costmap.
        void _compute_final_cost();

        /// @brief Fills a nav_msgs/OccupancyGrid from one grid_map layer, linearly mapping
        ///        [min_value, max_value] to the occupancy range [0, 100] and NaN cells to -1
        ///        (unknown). Built directly on grid_map_core's own getIndex(), rather than
        ///        grid_map_ros's converter -- see CMakeLists.txt for why.
        /// @param layer Name of the grid_map layer to convert.
        /// @param min_value Layer value mapped to occupancy 0.
        /// @param max_value Layer value mapped to occupancy 100.
        /// @param occupancy_grid_out Receives the converted grid.
        void _to_occupancy_grid(
            const std::string& layer,
            double min_value,
            double max_value,
            nav_msgs::msg::OccupancyGrid& occupancy_grid_out) const;

        /// @brief Publishes the cost layer as the final nav2-consumable occupancy costmap.
        /// @param stamp Timestamp to publish the message with.
        void _publish_costmap(const rclcpp::Time& stamp);

        /// @brief Publishes every debug layer (height_diff, steepness, roughness, ridge_bump,
        ///        ridge_pothole, clearance, border, obstacle, inflation) as its own OccupancyGrid,
        ///        for inspection/tuning in rviz.
        /// @param stamp Timestamp to publish the messages with.
        void _publish_layers(const rclcpp::Time& stamp);

        /// @brief One debug layer's name, occupancy value range and publisher.
        struct LayerPublisher
        {
            std::string layer;
            double min_value;
            double max_value;
            rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher;
        };

        std::string _pointcloud_topic{DEFAULT_POINTCLOUD_TOPIC};
        double _resolution_m{DEFAULT_RESOLUTION_M};
        double _map_margin_m{DEFAULT_MAP_MARGIN_M};

        double _neighbourhood_radius_m{DEFAULT_NEIGHBOURHOOD_RADIUS_M};
        int _min_plane_fit_points{DEFAULT_MIN_PLANE_FIT_POINTS};
        double _ground_margin_m{DEFAULT_GROUND_MARGIN_M};
        int _min_points_per_cell{DEFAULT_MIN_POINTS_PER_CELL};

        double _max_height_diff_m{DEFAULT_MAX_HEIGHT_DIFF_M};
        double _max_slope_deg{DEFAULT_MAX_SLOPE_DEG};
        double _max_roughness_m{DEFAULT_MAX_ROUGHNESS_M};
        double _min_clearance_m{DEFAULT_MIN_CLEARANCE_M};
        bool _treat_unknown_as_obstacle{DEFAULT_TREAT_UNKNOWN_AS_OBSTACLE};

        double _robot_radius_m{DEFAULT_ROBOT_RADIUS_M};
        double _inflation_radius_m{DEFAULT_INFLATION_RADIUS_M};
        double _cost_scaling_factor{DEFAULT_COST_SCALING_FACTOR};

        grid_map::GridMap _map;
        sensor_msgs::msg::PointCloud2::SharedPtr _latest_cloud;

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _pointcloud_subscription;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr _costmap_publisher;
        std::vector<LayerPublisher> _layer_publishers;
        rclcpp::TimerBase::SharedPtr _update_timer;
    };

}  // namespace global_traversability
