#include "pointcloudToLaserscan/PointToLaser.hpp"

#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace pointcloud_to_laserscan {

PointToLaser::PointToLaser(const rclcpp::NodeOptions& options) : rclcpp::Node("pointcloud_to_laserscan", options) {
    declare_parameter("param_topic_in", "livox/lidar");
    declare_parameter("param_topic_out", "scan");

    angle_min_ = this->declare_parameter("angle_min", -M_PI);
    angle_max_ = this->declare_parameter("angle_max", M_PI);
    angle_increment_ = this->declare_parameter("angle_increment", M_PI / 180.0);
    scan_time_ = this->declare_parameter("scan_time", 1.0 / 30.0);
    range_min_ = this->declare_parameter("range_min", 0.0);
    range_max_ = this->declare_parameter("range_max", std::numeric_limits<double>::max());

    param_topic_in_ = get_parameter("param_topic_in").as_string();
    param_topic_out_ = get_parameter("param_topic_out").as_string();

    const auto qos = rclcpp::SensorDataQoS(rclcpp::KeepLast(5)).reliable().durability_volatile();


    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        param_topic_in_, qos, std::bind(&PointToLaser::topic_callback, this, _1));
    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(param_topic_out_, rclcpp::SensorDataQoS());

    timer_ = this->create_wall_timer(10ms, std::bind(&PointToLaser::))
}

PointToLaser::~PointToLaser() {
}

void PointToLaser::topic_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
    sensor_msgs::msg::LaserScan::SharedPtr scan;
    scan->header = cloud->header;
    scan->angle_min = angle_min_;
    scan->angle_max = angle_max_;
    scan->angle_increment = angle_increment_;
    scan->time_increment = 0.0;
    scan->scan_time = scan_time_;
    scan->range_min = range_min_;
    scan->range_max = range_max_;

    uint32_t ranges_size = std::ceil(
        (scan->angle_max - scan->angle_min) / scan->angle_increment);
    scan_msg->ranges.assign(ranges_size, std::numeric_limits<double>::infinity());

    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x"),
        iter_y(*cloud, "y"), iter_z(*cloud, "z");
        iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
        if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) continue;
        if (*iter_z > max_height_ || *iter_z < min_height_) continue;

        double range = hypot(*iter_x, *iter_y);
        if (range < range_min_ || range > range_max_) continue;

        double angle = atan2(*iter_y, *iter_x);
        if (angle < scan_msg->angle_min || angle > scan_msg->angle_max) continue;

        int index = (angle - scan_msg->angle_min) / scan_msg->angle_increment;
        if (range < scan_msg->ranges[index]) scan_msg->ranges[index] = range;
        // TODO: intensities 추가
    }
    pub_->publish(std::move(scan));

}

}