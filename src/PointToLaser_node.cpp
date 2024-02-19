#include "pointcloudToLaserscan/PointToLaser.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointToLaser>());
    rclcpp::shutdown();
    return 0;
}