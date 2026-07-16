#include <cmath>
#include <cstdlib>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

double env_or_default(const char * name, double default_value)
{
  const char * value = std::getenv(name);
  if (!value) {
    return default_value;
  }
  try {
    return std::stod(value);
  } catch (const std::exception &) {
    return default_value;
  }
}

}  // namespace

class InitialPosePublisher : public rclcpp::Node
{
public:
  InitialPosePublisher() : Node("initial_pose_publisher")
  {
    declare_parameter<double>("x", 0.0);
    declare_parameter<double>("y", 0.0);
    declare_parameter<double>("yaw", 0.0);

    double x = get_parameter("x").as_double();
    double y = get_parameter("y").as_double();
    double yaw = get_parameter("yaw").as_double();

    if (x == 0.0 && y == 0.0 && yaw == 0.0) {
      x = env_or_default("INITIAL_POSE_X", 0.655);
      y = env_or_default("INITIAL_POSE_Y", 0.012);
      yaw = env_or_default("INITIAL_POSE_A", -3.119);
    }

    pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.frame_id = "map";
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation.z = std::sin(yaw / 2.0);
    msg.pose.pose.orientation.w = std::cos(yaw / 2.0);
    msg.pose.covariance[0] = 0.25;
    msg.pose.covariance[7] = 0.25;
    msg.pose.covariance[35] = 0.07;

    for (int i = 0; i < 5; ++i) {
      msg.header.stamp = now();
      pub_->publish(msg);
      rclcpp::sleep_for(std::chrono::milliseconds(200));
    }

    RCLCPP_INFO(get_logger(), "Published initial pose: x=%.3f, y=%.3f, yaw=%.3f", x, y, yaw);
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InitialPosePublisher>();
  node.reset();
  rclcpp::shutdown();
  return 0;
}
