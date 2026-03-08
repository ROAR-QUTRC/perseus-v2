#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <vector>

class SimPointcloudFilter : public rclcpp::Node
{
public:
    SimPointcloudFilter()
        : Node("sim_pointcloud_filter")
    {
        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar", 10,
            std::bind(&SimPointcloudFilter::callback, this, std::placeholders::_1));
        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar_filtered", 10);
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");

        std::vector<uint8_t> filtered_data;
        uint32_t count = 0;
        uint32_t point_step = msg->point_step;

        const uint8_t* data_ptr = msg->data.data();

        for (size_t i = 0; i < msg->width * msg->height; ++i, ++iter_x, ++iter_y, ++iter_z)
        {
            float x = *iter_x, y = *iter_y, z = *iter_z;
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
                continue;
            float dist = std::sqrt(x * x + y * y + z * z);
            if (dist < 0.1f)
                continue;
            const uint8_t* point = data_ptr + i * point_step;
            filtered_data.insert(filtered_data.end(), point, point + point_step);
            count++;
        }

        sensor_msgs::msg::PointCloud2 out;
        out.header = msg->header;
        out.fields = msg->fields;
        out.point_step = point_step;
        out.height = 1;
        out.width = count;
        out.row_step = count * point_step;
        out.is_dense = true;
        out.data = filtered_data;
        pub_->publish(out);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimPointcloudFilter>());
    rclcpp::shutdown();
    return 0;
}