#ifndef POINTCLOUD_TO_LASERSCAN_
#define POINTCLOUD_TO_LASERSCAN_

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace pointcloud_to_laserscan {

class PointToLaser : public rclcpp::Node {
public:
    explicit PointToLaser(const rclcpp::NodeOptions& options);
    ~PointToLaser();

protected:
    void topic_callback(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);

    std::string param_topic_in_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;

    std::string param_topic_out_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> pub_;
private:
    double min_height_, max_height_, angle_min_, angle_max_,
        angle_increment_, scan_time_, range_min_, range_max_;

    // rclcpp::TimerBase::SharedPtr timer_;
    // void node_loop(void);
};

}

#endif