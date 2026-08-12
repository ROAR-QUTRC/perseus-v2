/// @file global_traversability.cpp
/// @brief Implementation of the terrain-aware costmap generator.

#include "global_traversability/global_traversability/global_traversability.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

namespace global_traversability
{
    namespace
    {
        constexpr float NaN = std::numeric_limits<float>::quiet_NaN();

        /// @brief QoS depth used for the input map cloud subscription.
        constexpr int POINTCLOUD_QOS_DEPTH = 1;

        double deg_from_rad(double radians)
        {
            return radians * 180.0 / M_PI;
        }
    }  // namespace

    GlobalTraversability::GlobalTraversability(const rclcpp::NodeOptions& options)
        : rclcpp::Node("global_traversability", options),
          _map({"elevation_min", "elevation_max", "point_count", "height_diff", "steepness",
                "roughness", "ridge", "ridge_bump", "ridge_pothole", "clearance", "border",
                "obstacle", "inflation", "cost"})
    {
        _load_parameters();

        _pointcloud_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            _pointcloud_topic,
            rclcpp::QoS(POINTCLOUD_QOS_DEPTH),
            std::bind(&GlobalTraversability::_pointcloud_callback, this, std::placeholders::_1));

        // Transient local so nav2's static layer (or rviz, joining late) picks up the most
        // recent costmap immediately rather than waiting for the next update cycle.
        const auto latched_qos = rclcpp::QoS(1).transient_local();
        _costmap_publisher =
            this->create_publisher<nav_msgs::msg::OccupancyGrid>(DEFAULT_COSTMAP_TOPIC, latched_qos);

        // Debug/tuning layers, each its own OccupancyGrid (plain nav_msgs, no grid_map_rviz
        // plugin required) with a fixed value range chosen to make the layer's own units
        // legible. ridge itself is signed (a dip vs a bump) and is published split into two
        // unsigned layers instead of one signed one: rviz's occupancy colour schemes are built
        // for "0 = flat/uninteresting, 100 = extreme", so a single layer with flat sitting at the
        // signed midpoint (50) renders potholes and flat ground as visually indistinguishable,
        // while only bumps stand out. Splitting gives potholes the same full 0-100 range bumps
        // already had.
        const std::vector<LayerPublisher> layer_specs = {
            {"height_diff", 0.0, 1.0, nullptr},
            {"roughness", 0.0, 0.3, nullptr},
            {"steepness", 0.0, 90.0, nullptr},
            {"ridge_bump", 0.0, 0.5, nullptr},
            {"ridge_pothole", 0.0, 0.5, nullptr},
            {"clearance", 0.0, 2.0, nullptr},
            {"border", 0.0, 1.0, nullptr},
            {"obstacle", 0.0, 1.0, nullptr},
            {"inflation", 0.0, 99.0, nullptr},
        };
        for (const auto& spec : layer_specs)
        {
            _layer_publishers.push_back(
                {spec.layer,
                 spec.min_value,
                 spec.max_value,
                 this->create_publisher<nav_msgs::msg::OccupancyGrid>(
                     DEFAULT_LAYERS_TOPIC_PREFIX + spec.layer, latched_qos)});
        }

        const double update_period_s = this->get_parameter("update_period_s").as_double();
        const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(update_period_s));
        _update_timer = this->create_wall_timer(
            period, std::bind(&GlobalTraversability::_update_costmap, this));
    }

    void GlobalTraversability::_load_parameters()
    {
        _pointcloud_topic = this->declare_parameter("pointcloud_topic", DEFAULT_POINTCLOUD_TOPIC);
        _resolution_m = this->declare_parameter("resolution_m", DEFAULT_RESOLUTION_M);
        _map_margin_m = this->declare_parameter("map_margin_m", DEFAULT_MAP_MARGIN_M);

        _neighbourhood_radius_m =
            this->declare_parameter("neighbourhood_radius_m", DEFAULT_NEIGHBOURHOOD_RADIUS_M);
        _min_plane_fit_points =
            this->declare_parameter("min_plane_fit_points", DEFAULT_MIN_PLANE_FIT_POINTS);
        _ground_margin_m = this->declare_parameter("ground_margin_m", DEFAULT_GROUND_MARGIN_M);
        _min_points_per_cell =
            this->declare_parameter("min_points_per_cell", DEFAULT_MIN_POINTS_PER_CELL);

        _max_height_diff_m =
            this->declare_parameter("max_height_diff_m", DEFAULT_MAX_HEIGHT_DIFF_M);
        _max_slope_deg = this->declare_parameter("max_slope_deg", DEFAULT_MAX_SLOPE_DEG);
        _max_roughness_m = this->declare_parameter("max_roughness_m", DEFAULT_MAX_ROUGHNESS_M);
        _min_clearance_m = this->declare_parameter("min_clearance_m", DEFAULT_MIN_CLEARANCE_M);
        _treat_unknown_as_obstacle = this->declare_parameter(
            "treat_unknown_as_obstacle", DEFAULT_TREAT_UNKNOWN_AS_OBSTACLE);

        _robot_radius_m = this->declare_parameter("robot_radius_m", DEFAULT_ROBOT_RADIUS_M);
        _inflation_radius_m =
            this->declare_parameter("inflation_radius_m", DEFAULT_INFLATION_RADIUS_M);
        _cost_scaling_factor =
            this->declare_parameter("cost_scaling_factor", DEFAULT_COST_SCALING_FACTOR);

        this->declare_parameter("update_period_s", DEFAULT_UPDATE_PERIOD_S);
    }

    void GlobalTraversability::_pointcloud_callback(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Single-threaded executor by construction (no callback groups declared), so this never
        // races with _update_costmap on the wall timer -- a plain assignment is safe.
        _latest_cloud = msg;
    }

    void GlobalTraversability::_update_costmap()
    {
        if (!_latest_cloud)
        {
            return;
        }

        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*_latest_cloud, cloud);
        if (cloud.empty())
        {
            return;
        }

        float min_x = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float min_y = std::numeric_limits<float>::max();
        float max_y = std::numeric_limits<float>::lowest();
        for (const auto& point : cloud.points)
        {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
            {
                continue;
            }
            min_x = std::min(min_x, point.x);
            max_x = std::max(max_x, point.x);
            min_y = std::min(min_y, point.y);
            max_y = std::max(max_y, point.y);
        }
        if (min_x > max_x || min_y > max_y)
        {
            return;
        }

        const double length_x = static_cast<double>(max_x - min_x) + 2.0 * _map_margin_m;
        const double length_y = static_cast<double>(max_y - min_y) + 2.0 * _map_margin_m;
        const grid_map::Position center(
            (static_cast<double>(min_x) + max_x) / 2.0, (static_cast<double>(min_y) + max_y) / 2.0);

        _map.setGeometry(grid_map::Length(length_x, length_y), _resolution_m, center);
        _map.setFrameId(_latest_cloud->header.frame_id);

        _map["elevation_min"].setConstant(NaN);
        _map["elevation_max"].setConstant(NaN);
        _map["point_count"].setConstant(0.0f);
        _map["clearance"].setConstant(NaN);
        _map["steepness"].setConstant(NaN);
        _map["roughness"].setConstant(NaN);
        _map["ridge"].setConstant(NaN);
        _map["ridge_bump"].setConstant(NaN);
        _map["ridge_pothole"].setConstant(NaN);

        _accumulate_elevation(cloud);
        _compute_clearance(cloud);
        _map["height_diff"] = _map["elevation_max"] - _map["elevation_min"];
        _compute_local_terrain_features();
        _compute_border();
        _compute_obstacle();
        _compute_inflation();
        _compute_final_cost();

        const rclcpp::Time stamp(_latest_cloud->header.stamp);
        _publish_costmap(stamp);
        _publish_layers(stamp);
    }

    void GlobalTraversability::_accumulate_elevation(const pcl::PointCloud<pcl::PointXYZ>& cloud)
    {
        Eigen::MatrixXf& elevation_min = _map["elevation_min"];
        Eigen::MatrixXf& elevation_max = _map["elevation_max"];
        Eigen::MatrixXf& point_count = _map["point_count"];

        for (const auto& point : cloud.points)
        {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
            {
                continue;
            }

            grid_map::Index index;
            if (!_map.getIndex(grid_map::Position(point.x, point.y), index))
            {
                continue;
            }

            float& cell_min = elevation_min(index(0), index(1));
            float& cell_max = elevation_max(index(0), index(1));
            cell_min = std::isnan(cell_min) ? point.z : std::min(cell_min, point.z);
            cell_max = std::isnan(cell_max) ? point.z : std::max(cell_max, point.z);
            point_count(index(0), index(1)) += 1.0f;
        }
    }

    void GlobalTraversability::_compute_clearance(const pcl::PointCloud<pcl::PointXYZ>& cloud)
    {
        const Eigen::MatrixXf& elevation_min = _map["elevation_min"];
        Eigen::MatrixXf& clearance = _map["clearance"];
        const float ground_margin = static_cast<float>(_ground_margin_m);

        for (const auto& point : cloud.points)
        {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
            {
                continue;
            }

            grid_map::Index index;
            if (!_map.getIndex(grid_map::Position(point.x, point.y), index))
            {
                continue;
            }

            const float ground = elevation_min(index(0), index(1));
            if (std::isnan(ground) || point.z <= ground + ground_margin)
            {
                continue;
            }

            const float height_above_ground = point.z - ground;
            float& cell_clearance = clearance(index(0), index(1));
            cell_clearance =
                std::isnan(cell_clearance) ? height_above_ground : std::min(cell_clearance, height_above_ground);
        }
    }

    void GlobalTraversability::_compute_local_terrain_features()
    {
        const Eigen::MatrixXf& elevation = _map["elevation_min"];
        Eigen::MatrixXf& steepness = _map["steepness"];
        Eigen::MatrixXf& roughness = _map["roughness"];
        Eigen::MatrixXf& ridge = _map["ridge"];
        Eigen::MatrixXf& ridge_bump = _map["ridge_bump"];
        Eigen::MatrixXf& ridge_pothole = _map["ridge_pothole"];

        const int rows = _map.getSize()(0);
        const int cols = _map.getSize()(1);
        const int window_cells =
            std::max(1, static_cast<int>(std::round(_neighbourhood_radius_m / _map.getResolution())));

        std::vector<Eigen::Vector3d> neighbours;
        for (grid_map::GridMapIterator it(_map); !it.isPastEnd(); ++it)
        {
            const grid_map::Index index(*it);
            const int row = index(0);
            const int col = index(1);

            const float center = elevation(row, col);
            if (std::isnan(center))
            {
                continue;
            }

            neighbours.clear();
            for (int d_row = -window_cells; d_row <= window_cells; ++d_row)
            {
                const int neighbour_row = row + d_row;
                if (neighbour_row < 0 || neighbour_row >= rows)
                {
                    continue;
                }
                for (int d_col = -window_cells; d_col <= window_cells; ++d_col)
                {
                    const int neighbour_col = col + d_col;
                    if (neighbour_col < 0 || neighbour_col >= cols)
                    {
                        continue;
                    }

                    const float z = elevation(neighbour_row, neighbour_col);
                    if (std::isnan(z))
                    {
                        continue;
                    }

                    grid_map::Position position;
                    _map.getPosition(grid_map::Index(neighbour_row, neighbour_col), position);
                    neighbours.emplace_back(position.x(), position.y(), static_cast<double>(z));
                }
            }

            if (static_cast<int>(neighbours.size()) < _min_plane_fit_points)
            {
                continue;
            }

            Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
            for (const auto& neighbour : neighbours)
            {
                centroid += neighbour;
            }
            centroid /= static_cast<double>(neighbours.size());

            Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
            for (const auto& neighbour : neighbours)
            {
                const Eigen::Vector3d diff = neighbour - centroid;
                covariance += diff * diff.transpose();
            }
            covariance /= static_cast<double>(neighbours.size());

            // The eigenvector of the smallest eigenvalue is the best-fit plane's normal; that
            // eigenvalue itself is the variance of points off the plane, i.e. the roughness.
            const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
            const Eigen::Vector3d normal = solver.eigenvectors().col(0);
            const double residual_variance = std::max(0.0, solver.eigenvalues()(0));

            steepness(row, col) =
                static_cast<float>(deg_from_rad(std::acos(std::min(1.0, std::abs(normal.z())))));
            roughness(row, col) = static_cast<float>(std::sqrt(residual_variance));

            const float ridge_value = static_cast<float>(static_cast<double>(center) - centroid.z());
            ridge(row, col) = ridge_value;
            // Split into two unsigned layers rather than publishing this signed value directly:
            // see the layer_specs comment in the constructor for why.
            ridge_bump(row, col) = std::max(0.0f, ridge_value);
            ridge_pothole(row, col) = std::max(0.0f, -ridge_value);
        }
    }

    void GlobalTraversability::_compute_border()
    {
        const Eigen::MatrixXf& point_count = _map["point_count"];
        Eigen::MatrixXf& border = _map["border"];
        border = (point_count.array() < static_cast<float>(_min_points_per_cell)).cast<float>().matrix();
    }

    void GlobalTraversability::_compute_obstacle()
    {
        const Eigen::MatrixXf& height_diff = _map["height_diff"];
        const Eigen::MatrixXf& steepness = _map["steepness"];
        const Eigen::MatrixXf& roughness = _map["roughness"];
        const Eigen::MatrixXf& clearance = _map["clearance"];
        const Eigen::MatrixXf& border = _map["border"];
        Eigen::MatrixXf& obstacle = _map["obstacle"];

        const float max_height_diff = static_cast<float>(_max_height_diff_m);
        const float max_slope = static_cast<float>(_max_slope_deg);
        const float max_roughness = static_cast<float>(_max_roughness_m);
        const float min_clearance = static_cast<float>(_min_clearance_m);

        const int rows = _map.getSize()(0);
        const int cols = _map.getSize()(1);
        for (int row = 0; row < rows; ++row)
        {
            for (int col = 0; col < cols; ++col)
            {
                if (border(row, col) > 0.5f)
                {
                    obstacle(row, col) = _treat_unknown_as_obstacle ? 1.0f : 0.0f;
                    continue;
                }

                const float height_diff_value = height_diff(row, col);
                const float steepness_value = steepness(row, col);
                const float roughness_value = roughness(row, col);
                const float clearance_value = clearance(row, col);

                const bool is_obstacle =
                    (!std::isnan(height_diff_value) && height_diff_value > max_height_diff) ||
                    (!std::isnan(steepness_value) && steepness_value > max_slope) ||
                    (!std::isnan(roughness_value) && roughness_value > max_roughness) ||
                    (!std::isnan(clearance_value) && clearance_value < min_clearance);

                obstacle(row, col) = is_obstacle ? 1.0f : 0.0f;
            }
        }
    }

    void GlobalTraversability::_compute_inflation()
    {
        const Eigen::MatrixXf& obstacle = _map["obstacle"];
        Eigen::MatrixXf& inflation = _map["inflation"];

        const int rows = _map.getSize()(0);
        const int cols = _map.getSize()(1);
        const float resolution = static_cast<float>(_map.getResolution());
        const float inf = std::numeric_limits<float>::infinity();
        const float diagonal = static_cast<float>(std::sqrt(2.0));

        // Two-pass chamfer distance transform: an approximate Euclidean distance-to-nearest-
        // obstacle in cell units, cheap enough to rerun on every update without extra deps.
        Eigen::MatrixXf distance = Eigen::MatrixXf::Constant(rows, cols, inf);
        for (int row = 0; row < rows; ++row)
        {
            for (int col = 0; col < cols; ++col)
            {
                if (obstacle(row, col) > 0.5f)
                {
                    distance(row, col) = 0.0f;
                }
            }
        }

        for (int row = 0; row < rows; ++row)
        {
            for (int col = 0; col < cols; ++col)
            {
                float& d = distance(row, col);
                if (row > 0)
                {
                    d = std::min(d, distance(row - 1, col) + 1.0f);
                }
                if (col > 0)
                {
                    d = std::min(d, distance(row, col - 1) + 1.0f);
                }
                if (row > 0 && col > 0)
                {
                    d = std::min(d, distance(row - 1, col - 1) + diagonal);
                }
                if (row > 0 && col + 1 < cols)
                {
                    d = std::min(d, distance(row - 1, col + 1) + diagonal);
                }
            }
        }
        for (int row = rows - 1; row >= 0; --row)
        {
            for (int col = cols - 1; col >= 0; --col)
            {
                float& d = distance(row, col);
                if (row + 1 < rows)
                {
                    d = std::min(d, distance(row + 1, col) + 1.0f);
                }
                if (col + 1 < cols)
                {
                    d = std::min(d, distance(row, col + 1) + 1.0f);
                }
                if (row + 1 < rows && col + 1 < cols)
                {
                    d = std::min(d, distance(row + 1, col + 1) + diagonal);
                }
                if (row + 1 < rows && col > 0)
                {
                    d = std::min(d, distance(row + 1, col - 1) + diagonal);
                }
            }
        }

        // Matches nav2's InflationLayer convention: lethal out to the robot's own radius (it
        // cannot fit its center any closer to an obstacle than that), then an exponential decay
        // out to inflation_radius_m, then clear.
        const float inscribed_radius_cells = static_cast<float>(_robot_radius_m) / resolution;
        const float inflation_radius_cells = static_cast<float>(_inflation_radius_m) / resolution;
        const float scaling_factor = static_cast<float>(_cost_scaling_factor);
        const float robot_radius = static_cast<float>(_robot_radius_m);

        for (int row = 0; row < rows; ++row)
        {
            for (int col = 0; col < cols; ++col)
            {
                const float distance_cells = distance(row, col);
                if (distance_cells <= inscribed_radius_cells)
                {
                    inflation(row, col) = 99.0f;
                }
                else if (distance_cells <= inflation_radius_cells)
                {
                    const float distance_m = distance_cells * resolution;
                    inflation(row, col) = 99.0f * std::exp(-scaling_factor * (distance_m - robot_radius));
                }
                else
                {
                    inflation(row, col) = 0.0f;
                }
            }
        }
    }

    void GlobalTraversability::_compute_final_cost()
    {
        const Eigen::MatrixXf& obstacle = _map["obstacle"];
        const Eigen::MatrixXf& inflation = _map["inflation"];
        const Eigen::MatrixXf& border = _map["border"];
        Eigen::MatrixXf& cost = _map["cost"];

        const int rows = _map.getSize()(0);
        const int cols = _map.getSize()(1);
        for (int row = 0; row < rows; ++row)
        {
            for (int col = 0; col < cols; ++col)
            {
                if (border(row, col) > 0.5f && !_treat_unknown_as_obstacle)
                {
                    cost(row, col) = NaN;  // -> -1 (unknown) once exported as an OccupancyGrid.
                    continue;
                }
                cost(row, col) = obstacle(row, col) > 0.5f ? 100.0f : std::min(99.0f, inflation(row, col));
            }
        }
    }

    void GlobalTraversability::_to_occupancy_grid(
        const std::string& layer,
        double min_value,
        double max_value,
        nav_msgs::msg::OccupancyGrid& occupancy_grid_out) const
    {
        const double resolution = _map.getResolution();
        const grid_map::Length length = _map.getLength();
        const grid_map::Position center = _map.getPosition();

        const int width = std::max(1, static_cast<int>(std::round(length.x() / resolution)));
        const int height = std::max(1, static_cast<int>(std::round(length.y() / resolution)));

        occupancy_grid_out.header.frame_id = _map.getFrameId();
        occupancy_grid_out.info.resolution = static_cast<float>(resolution);
        occupancy_grid_out.info.width = static_cast<uint32_t>(width);
        occupancy_grid_out.info.height = static_cast<uint32_t>(height);
        occupancy_grid_out.info.origin.position.x = center.x() - length.x() / 2.0;
        occupancy_grid_out.info.origin.position.y = center.y() - length.y() / 2.0;
        occupancy_grid_out.info.origin.position.z = 0.0;
        occupancy_grid_out.info.origin.orientation.w = 1.0;

        occupancy_grid_out.data.assign(static_cast<size_t>(width) * static_cast<size_t>(height), -1);

        const Eigen::MatrixXf& values = _map[layer];
        const double range = max_value - min_value;

        // Iterate OccupancyGrid cells (a convention we fully control) rather than grid_map's own
        // row/column order, and ask grid_map's own getIndex() for the matching cell each time --
        // that way this never has to know or reimplement grid_map's internal index<->world
        // convention, only trust the same lookup already used everywhere else in this file.
        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                const double world_x = occupancy_grid_out.info.origin.position.x + (x + 0.5) * resolution;
                const double world_y = occupancy_grid_out.info.origin.position.y + (y + 0.5) * resolution;

                grid_map::Index index;
                if (!_map.getIndex(grid_map::Position(world_x, world_y), index))
                {
                    continue;  // stays -1 (unknown)
                }

                const float value = values(index(0), index(1));
                if (std::isnan(value))
                {
                    continue;  // stays -1 (unknown)
                }

                const double normalised = range > 0.0 ? (static_cast<double>(value) - min_value) / range : 0.0;
                const long occupancy = std::lround(std::clamp(normalised, 0.0, 1.0) * 100.0);
                occupancy_grid_out.data[static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x)] =
                    static_cast<int8_t>(occupancy);
            }
        }
    }

    void GlobalTraversability::_publish_costmap(const rclcpp::Time& stamp)
    {
        nav_msgs::msg::OccupancyGrid occupancy_grid;
        _to_occupancy_grid("cost", 0.0, 100.0, occupancy_grid);
        occupancy_grid.header.stamp = stamp;
        _costmap_publisher->publish(occupancy_grid);
    }

    void GlobalTraversability::_publish_layers(const rclcpp::Time& stamp)
    {
        for (const auto& layer_publisher : _layer_publishers)
        {
            nav_msgs::msg::OccupancyGrid occupancy_grid;
            _to_occupancy_grid(
                layer_publisher.layer, layer_publisher.min_value, layer_publisher.max_value, occupancy_grid);
            occupancy_grid.header.stamp = stamp;
            layer_publisher.publisher->publish(occupancy_grid);
        }
    }

}  // namespace global_traversability
