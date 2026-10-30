#include <rclcpp/rclcpp.hpp>
#include <pcl_perception/pcl_perception_node.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pcl_perception::PclPerception>());
    rclcpp::shutdown();
    return 0;
}