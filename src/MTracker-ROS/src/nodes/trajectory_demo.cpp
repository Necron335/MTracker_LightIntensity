#include <chrono>
#include <exception>
#include <memory>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include "mtracker/control/demo_control.hpp"

namespace mtracker
{

class TrajectoryNode : public rclcpp::Node
{
public:
    TrajectoryNode()
    : Node("mtracker_trajectory"),
      trail_(declare_parameter<int>("max_points", 10000),
        declare_parameter<double>("min_distance", 0.002))
    {
        publisher_ = create_publisher<nav_msgs::msg::Path>(
            "trajectory", rclcpp::QoS(1).transient_local());
        subscription_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = message->header;
                pose.pose = message->pose.pose;
                trail_.append(pose);
            });
        timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
                if (!trail_.path().poses.empty()) {
                    publisher_->publish(trail_.path());
                }
            });
    }

private:
    TrajectoryBuffer trail_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mtracker

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    int result = 0;
    try {
        rclcpp::spin(std::make_shared<mtracker::TrajectoryNode>());
    } catch (const std::exception & error) {
        RCLCPP_FATAL(rclcpp::get_logger("mtracker_trajectory"), "%s", error.what());
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
