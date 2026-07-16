#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <cctype>

#include <yaml-cpp/yaml.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{

double clamp_value(double value, double low, double high)
{
  return std::max(low, std::min(high, value));
}

double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

bool as_bool(const rclcpp::Parameter & parameter)
{
  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
    return parameter.as_bool();
  }
  const auto text = parameter.value_to_string();
  return text == "1" || text == "true" || text == "True" || text == "TRUE" ||
         text == "yes" || text == "on" || text == "ON";
}

double as_double(const rclcpp::Parameter & parameter, double default_value = 0.0)
{
  try {
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      return parameter.as_double();
    }
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      return static_cast<double>(parameter.as_int());
    }
    return std::stod(parameter.value_to_string());
  } catch (const std::exception &) {
    return default_value;
  }
}

int as_int(const rclcpp::Parameter & parameter, int default_value = 0)
{
  try {
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      return static_cast<int>(parameter.as_int());
    }
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      return static_cast<int>(parameter.as_double());
    }
    return std::stoi(parameter.value_to_string());
  } catch (const std::exception &) {
    return default_value;
  }
}

bool yaml_bool(const YAML::Node & node, bool default_value = false)
{
  if (!node) {
    return default_value;
  }
  try {
    return node.as<bool>();
  } catch (const YAML::Exception &) {
    try {
      const auto text = node.as<std::string>();
      return text == "1" || text == "true" || text == "True" || text == "TRUE" ||
             text == "yes" || text == "on" || text == "ON";
    } catch (const YAML::Exception &) {
      return default_value;
    }
  }
}

struct Waypoint
{
  double x{0.0};
  double y{0.0};
  bool reverse{false};
  double pass_radius{0.0};
  double reverse_pass_radius{0.0};
};

struct TrackingTarget
{
  double x{0.0};
  double y{0.0};
  bool reverse{false};
};

std::string to_lower(std::string text)
{
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return text;
}

}  // namespace

class SmoothPathFollower : public rclcpp::Node
{
public:
  SmoothPathFollower()
  : Node("smooth_path_follower"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameter<std::string>("waypoints_file", "");
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<std::string>("base_frame_id", "base_link");
    declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    declare_parameter<std::string>("scan_topic", "/scan");
    declare_parameter<std::string>("keepout_mask_topic", "/filter_mask");
    declare_parameter<double>("control_frequency", 20.0);
    declare_parameter<double>("linear_speed", 0.35);
    declare_parameter<double>("channel_linear_speed", 0.45);
    declare_parameter<std::string>("channel_waypoint_ranges", "");
    declare_parameter<double>("lookahead_distance", 0.55);
    declare_parameter<double>("pass_radius", 0.35);
    rcl_interfaces::msg::ParameterDescriptor dynamic_parameter;
    dynamic_parameter.dynamic_typing = true;
    declare_parameter("enable_smooth_keepout_path", rclcpp::ParameterValue(true), dynamic_parameter);
    declare_parameter("smooth_path_min_waypoint", rclcpp::ParameterValue(4), dynamic_parameter);
    declare_parameter("smooth_path_max_waypoint", rclcpp::ParameterValue(9), dynamic_parameter);
    declare_parameter("smooth_path_samples_per_segment", rclcpp::ParameterValue(10), dynamic_parameter);
    declare_parameter("smooth_path_keepout_clearance", rclcpp::ParameterValue(0.22), dynamic_parameter);
    declare_parameter("smooth_path_keepout_weight", rclcpp::ParameterValue(0.9), dynamic_parameter);
    declare_parameter<bool>("enable_skip_overshot_waypoints", true);
    declare_parameter("overshot_waypoint_margin", rclcpp::ParameterValue(0.05), dynamic_parameter);
    declare_parameter("overshot_waypoint_max_distance", rclcpp::ParameterValue(0.80), dynamic_parameter);
    declare_parameter("overshot_next_waypoint_max_distance", rclcpp::ParameterValue(0.45), dynamic_parameter);
    declare_parameter("skip_overshot_min_waypoint", rclcpp::ParameterValue(5), dynamic_parameter);
    declare_parameter("skip_overshot_max_waypoint", rclcpp::ParameterValue(9), dynamic_parameter);
    declare_parameter<double>("goal_tolerance", 0.25);
    declare_parameter<double>("max_angular_z", 1.6);
    declare_parameter<double>("turn_angular_gain", 1.4);
    declare_parameter<double>("turn_min_speed_scale", 0.45);
    declare_parameter<double>("reverse_angular_gain", 1.1);
    declare_parameter<double>("reverse_min_speed_scale", 0.30);
    declare_parameter<double>("reverse_goal_lookahead_distance", 0.25);
    declare_parameter<double>("reverse_pass_radius", 0.25);
    declare_parameter("enable_reverse_approach_nudge", rclcpp::ParameterValue(true), dynamic_parameter);
    declare_parameter("reverse_approach_nudge_waypoint", rclcpp::ParameterValue(2), dynamic_parameter);
    declare_parameter("reverse_approach_nudge_distance", rclcpp::ParameterValue(0.35), dynamic_parameter);
    declare_parameter("reverse_approach_nudge_angular_z", rclcpp::ParameterValue(-0.35), dynamic_parameter);
    declare_parameter<bool>("enable_obstacle_avoidance", true);
    declare_parameter<int>("obstacle_enable_from_waypoint", 0);
    declare_parameter<double>("obstacle_stop_distance", 0.22);
    declare_parameter<double>("obstacle_slow_distance", 0.80);
    declare_parameter<double>("obstacle_avoid_distance", 0.60);
    declare_parameter<double>("min_obstacle_range", 0.10);
    declare_parameter<double>("front_angle_deg", 30.0);
    declare_parameter<double>("side_angle_deg", 90.0);
    declare_parameter<double>("avoid_angular_z", 0.55);
    declare_parameter("enable_keepout_avoidance", rclcpp::ParameterValue(true), dynamic_parameter);
    declare_parameter("keepout_occupied_threshold", rclcpp::ParameterValue(50), dynamic_parameter);
    declare_parameter("keepout_stop_distance", rclcpp::ParameterValue(0.25), dynamic_parameter);
    declare_parameter("keepout_slow_distance", rclcpp::ParameterValue(0.70), dynamic_parameter);
    declare_parameter("keepout_avoid_distance", rclcpp::ParameterValue(0.55), dynamic_parameter);
    declare_parameter("keepout_side_sample_distance", rclcpp::ParameterValue(0.35), dynamic_parameter);
    declare_parameter("enable_keepout_hard_stop", rclcpp::ParameterValue(false), dynamic_parameter);
    declare_parameter("keepout_min_speed_scale", rclcpp::ParameterValue(0.45), dynamic_parameter);
    declare_parameter<double>("backup_trigger_time", 0.0);
    declare_parameter<double>("backup_duration", 0.5);
    declare_parameter<double>("backup_speed", 0.12);
    declare_parameter<double>("backup_angular_z", 0.35);
    declare_parameter<double>("backup_stop_distance", 0.18);
    declare_parameter<bool>("enable_qr_skip_first", true);
    declare_parameter<std::string>("qr_topic", "/qr_info");
    declare_parameter<std::string>("route_direction", "auto");
    declare_parameter<bool>("enable_qr_direction_select", true);
    declare_parameter<bool>("use_qr_parity_direction", true);
    declare_parameter<int>("clockwise_qr_value", 1);
    declare_parameter<int>("counterclockwise_qr_value", 2);
    declare_parameter<bool>("enable_image_analysis_trigger", true);
    declare_parameter<int>("image_analysis_waypoint", 7);
    declare_parameter<std::string>("image_analysis_trigger_topic", "/person_trigger");

    waypoints_file_ = get_parameter("waypoints_file").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    base_frame_id_ = get_parameter("base_frame_id").as_string();
    linear_speed_ = get_parameter("linear_speed").as_double();
    channel_linear_speed_ = get_parameter("channel_linear_speed").as_double();
    channel_waypoint_ranges_ = parse_waypoint_ranges(get_parameter("channel_waypoint_ranges").as_string());
    lookahead_distance_ = get_parameter("lookahead_distance").as_double();
    pass_radius_ = get_parameter("pass_radius").as_double();
    enable_smooth_keepout_path_ = as_bool(get_parameter("enable_smooth_keepout_path"));
    smooth_path_min_waypoint_ =
      std::max(1, as_int(get_parameter("smooth_path_min_waypoint"), 4));
    smooth_path_max_waypoint_ =
      std::max(smooth_path_min_waypoint_, as_int(get_parameter("smooth_path_max_waypoint"), 9));
    smooth_path_samples_per_segment_ =
      std::max(3, as_int(get_parameter("smooth_path_samples_per_segment"), 10));
    smooth_path_keepout_clearance_ =
      std::max(0.0, as_double(get_parameter("smooth_path_keepout_clearance"), 0.22));
    smooth_path_keepout_weight_ =
      std::max(0.0, as_double(get_parameter("smooth_path_keepout_weight"), 0.9));
    enable_skip_overshot_waypoints_ = as_bool(get_parameter("enable_skip_overshot_waypoints"));
    overshot_waypoint_margin_ =
      std::max(0.0, as_double(get_parameter("overshot_waypoint_margin"), 0.05));
    overshot_waypoint_max_distance_ =
      std::max(0.0, as_double(get_parameter("overshot_waypoint_max_distance"), 0.80));
    overshot_next_waypoint_max_distance_ =
      std::max(0.0, as_double(get_parameter("overshot_next_waypoint_max_distance"), 0.45));
    skip_overshot_min_waypoint_ =
      std::max(1, as_int(get_parameter("skip_overshot_min_waypoint"), 5));
    skip_overshot_max_waypoint_ =
      std::max(skip_overshot_min_waypoint_, as_int(get_parameter("skip_overshot_max_waypoint"), 9));
    goal_tolerance_ = get_parameter("goal_tolerance").as_double();
    max_angular_z_ = get_parameter("max_angular_z").as_double();
    turn_angular_gain_ = get_parameter("turn_angular_gain").as_double();
    turn_min_speed_scale_ = get_parameter("turn_min_speed_scale").as_double();
    reverse_angular_gain_ = get_parameter("reverse_angular_gain").as_double();
    reverse_min_speed_scale_ = get_parameter("reverse_min_speed_scale").as_double();
    reverse_goal_lookahead_distance_ = get_parameter("reverse_goal_lookahead_distance").as_double();
    reverse_pass_radius_ = get_parameter("reverse_pass_radius").as_double();
    enable_reverse_approach_nudge_ = as_bool(get_parameter("enable_reverse_approach_nudge"));
    reverse_approach_nudge_waypoint_ =
      std::max(1, as_int(get_parameter("reverse_approach_nudge_waypoint"), 2));
    reverse_approach_nudge_distance_ =
      std::max(0.0, as_double(get_parameter("reverse_approach_nudge_distance"), 0.35));
    reverse_approach_nudge_angular_z_ =
      as_double(get_parameter("reverse_approach_nudge_angular_z"), -0.35);
    enable_obstacle_avoidance_ = as_bool(get_parameter("enable_obstacle_avoidance"));
    obstacle_enable_from_waypoint_ = std::max(0, static_cast<int>(get_parameter("obstacle_enable_from_waypoint").as_int()));
    obstacle_stop_distance_ = get_parameter("obstacle_stop_distance").as_double();
    obstacle_slow_distance_ = get_parameter("obstacle_slow_distance").as_double();
    obstacle_avoid_distance_ = get_parameter("obstacle_avoid_distance").as_double();
    min_obstacle_range_ = get_parameter("min_obstacle_range").as_double();
    front_angle_ = get_parameter("front_angle_deg").as_double() * M_PI / 180.0;
    side_angle_ = get_parameter("side_angle_deg").as_double() * M_PI / 180.0;
    avoid_angular_z_ = get_parameter("avoid_angular_z").as_double();
    enable_keepout_avoidance_ = as_bool(get_parameter("enable_keepout_avoidance"));
    keepout_occupied_threshold_ =
      std::max(1, std::min(100, as_int(get_parameter("keepout_occupied_threshold"), 50)));
    keepout_stop_distance_ =
      std::max(0.0, as_double(get_parameter("keepout_stop_distance"), 0.25));
    keepout_slow_distance_ =
      std::max(keepout_stop_distance_, as_double(get_parameter("keepout_slow_distance"), 0.70));
    keepout_avoid_distance_ =
      std::max(keepout_stop_distance_, as_double(get_parameter("keepout_avoid_distance"), 0.55));
    keepout_side_sample_distance_ =
      std::max(0.05, as_double(get_parameter("keepout_side_sample_distance"), 0.35));
    enable_keepout_hard_stop_ = as_bool(get_parameter("enable_keepout_hard_stop"));
    keepout_min_speed_scale_ = clamp_value(
      as_double(get_parameter("keepout_min_speed_scale"), 0.45), 0.05, 1.0);
    backup_trigger_time_ = get_parameter("backup_trigger_time").as_double();
    backup_duration_ = get_parameter("backup_duration").as_double();
    backup_speed_ = get_parameter("backup_speed").as_double();
    backup_angular_z_ = get_parameter("backup_angular_z").as_double();
    backup_stop_distance_ = get_parameter("backup_stop_distance").as_double();
    enable_qr_skip_first_ = as_bool(get_parameter("enable_qr_skip_first"));
    qr_topic_ = get_parameter("qr_topic").as_string();
    route_direction_ = to_lower(get_parameter("route_direction").as_string());
    enable_qr_direction_select_ = as_bool(get_parameter("enable_qr_direction_select"));
    use_qr_parity_direction_ = as_bool(get_parameter("use_qr_parity_direction"));
    clockwise_qr_value_ = static_cast<int>(get_parameter("clockwise_qr_value").as_int());
    counterclockwise_qr_value_ = static_cast<int>(get_parameter("counterclockwise_qr_value").as_int());
    enable_image_analysis_trigger_ = as_bool(get_parameter("enable_image_analysis_trigger"));
    image_analysis_waypoint_ = std::max(1, static_cast<int>(get_parameter("image_analysis_waypoint").as_int()));
    image_analysis_trigger_topic_ = get_parameter("image_analysis_trigger_topic").as_string();
    const double control_frequency = get_parameter("control_frequency").as_double();

    waypoints_ = load_waypoints(route_direction_);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(get_parameter("cmd_vel_topic").as_string(), 10);
    image_analysis_trigger_pub_ =
      create_publisher<std_msgs::msg::Int32>(image_analysis_trigger_topic_, rclcpp::QoS(10).transient_local());
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      get_parameter("scan_topic").as_string(), 10,
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) { latest_scan_ = msg; });
    keepout_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      get_parameter("keepout_mask_topic").as_string(), rclcpp::QoS(1).transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) { latest_keepout_mask_ = msg; });

    if (enable_qr_skip_first_ || enable_qr_direction_select_) {
      qr_sub_ = create_subscription<std_msgs::msg::String>(
        qr_topic_, 10, std::bind(&SmoothPathFollower::qr_callback, this, std::placeholders::_1));
    }

    if (waypoints_.empty()) {
      RCLCPP_ERROR(get_logger(), "No waypoints loaded; smooth path follower will stop.");
      publish_stop();
      return;
    }

    const auto reverse_count = std::count_if(
      waypoints_.begin(), waypoints_.end(), [](const Waypoint & w) { return w.reverse; });
    RCLCPP_INFO(
      get_logger(),
      "Loaded %zu waypoint(s) (%zu reverse), route=%s, lookahead=%.2fm, pass_radius=%.2fm, outside_speed=%.2fm/s, channel_speed=%.2fm/s, obstacle_from_waypoint=%d, obstacle_avoidance=%s.",
      waypoints_.size(), static_cast<std::size_t>(reverse_count), selected_route_direction_.c_str(), lookahead_distance_, pass_radius_,
      linear_speed_, channel_linear_speed_, obstacle_enable_from_waypoint_,
      enable_obstacle_avoidance_ ? "true" : "false");

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(control_frequency, 1.0)),
      std::bind(&SmoothPathFollower::control_once, this));
  }

  void publish_stop()
  {
    if (cmd_pub_) {
      cmd_pub_->publish(geometry_msgs::msg::Twist());
    }
  }

private:
  std::vector<std::pair<int, int>> parse_waypoint_ranges(std::string value)
  {
    for (std::size_t pos = value.find("，"); pos != std::string::npos; pos = value.find("，", pos + 1)) {
      value.replace(pos, std::string("，").size(), ",");
    }
    std::vector<std::pair<int, int>> ranges;
    std::size_t start_pos = 0;
    while (start_pos <= value.size()) {
      const auto comma = value.find(',', start_pos);
      std::string part = value.substr(start_pos, comma == std::string::npos ? std::string::npos : comma - start_pos);
      part.erase(std::remove_if(part.begin(), part.end(), ::isspace), part.end());
      if (!part.empty()) {
        try {
          int first = 0;
          int last = 0;
          const auto dash = part.find('-');
          if (dash == std::string::npos) {
            first = last = std::stoi(part);
          } else {
            first = std::stoi(part.substr(0, dash));
            last = std::stoi(part.substr(dash + 1));
          }
          if (first > last) {
            std::swap(first, last);
          }
          ranges.emplace_back(std::max(first, 1), std::max(last, 1));
        } catch (const std::exception &) {
          RCLCPP_WARN(get_logger(), "Ignoring invalid channel waypoint range: %s", part.c_str());
        }
      }
      if (comma == std::string::npos) {
        break;
      }
      start_pos = comma + 1;
    }
    return ranges;
  }

  void qr_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string direction = direction_from_qr(msg->data);
    if (!direction.empty()) {
      switch_route(direction);
    }

    if (enable_qr_skip_first_) {
      skip_first_waypoint_from_qr(msg->data);
    }
  }

  std::string direction_from_qr(const std::string & data) const
  {
    if (!enable_qr_direction_select_) {
      return "";
    }
    int value = 0;
    try {
      value = std::stoi(data);
    } catch (const std::exception &) {
      return "";
    }
    if (use_qr_parity_direction_) {
      return value % 2 == 0 ? "counterclockwise" : "clockwise";
    }
    if (value == clockwise_qr_value_) {
      return "clockwise";
    }
    if (value == counterclockwise_qr_value_) {
      return "counterclockwise";
    }
    return "";
  }

  void switch_route(const std::string & direction)
  {
    if (direction == selected_route_direction_) {
      return;
    }

    const auto new_waypoints = load_waypoints(direction);
    if (new_waypoints.empty()) {
      RCLCPP_WARN(get_logger(), "QR selected route \"%s\", but no waypoints were loaded.", direction.c_str());
      return;
    }

    waypoints_ = new_waypoints;
    selected_route_direction_ = direction;
    current_index_ = std::min(current_index_, waypoints_.size() - 1);
    emergency_start_time_ = std::numeric_limits<double>::quiet_NaN();
    backup_until_time_ = std::numeric_limits<double>::quiet_NaN();
    RCLCPP_INFO(
      get_logger(), "QR selected %s route; continuing from waypoint %zu/%zu.",
      selected_route_direction_.c_str(), current_index_ + 1, waypoints_.size());
  }

  void skip_first_waypoint_from_qr(const std::string & data)
  {
    if (qr_skip_done_) {
      return;
    }
    if (waypoints_.size() < 2) {
      qr_skip_done_ = true;
      RCLCPP_WARN(get_logger(), "QR skip ignored: fewer than 2 waypoints loaded.");
      return;
    }
    if (current_index_ == 0) {
      current_index_ = 1;
      qr_skip_done_ = true;
      emergency_start_time_ = std::numeric_limits<double>::quiet_NaN();
      backup_until_time_ = std::numeric_limits<double>::quiet_NaN();
      RCLCPP_INFO(
        get_logger(), "QR info received \"%s\"; switching directly to waypoint 2 (%s).",
        data.c_str(), waypoints_[current_index_].reverse ? "reverse" : "forward");
      return;
    }
    qr_skip_done_ = true;
    RCLCPP_INFO(get_logger(), "QR info received \"%s\", but waypoint 1 is already passed; no skip needed.", data.c_str());
  }

  std::vector<Waypoint> load_waypoints(const std::string & requested_direction)
  {
    std::vector<Waypoint> waypoints;
    std::string resolved_direction = requested_direction;
    if (waypoints_file_.empty()) {
      RCLCPP_ERROR(get_logger(), "Parameter waypoints_file is empty.");
      return waypoints;
    }
    try {
      const YAML::Node data = YAML::LoadFile(waypoints_file_);
      std::string direction = to_lower(requested_direction);
      if (direction.empty() || direction == "auto") {
        direction = data["route_direction"] ? to_lower(data["route_direction"].as<std::string>()) : "counterclockwise";
      }
      resolved_direction = direction;
      const YAML::Node routes = data["routes"];
      const YAML::Node items = routes && routes[direction] ? routes[direction] : data["waypoints"];
      if (!items || !items.IsSequence()) {
        return waypoints;
      }
      for (std::size_t i = 0; i < items.size(); ++i) {
        try {
          Waypoint w;
          w.x = items[i]["x"].as<double>();
          w.y = items[i]["y"].as<double>();
          w.reverse = yaml_bool(items[i]["reverse"], false);
          w.pass_radius = items[i]["pass_radius"] ? items[i]["pass_radius"].as<double>() : pass_radius_;
          w.reverse_pass_radius =
            items[i]["reverse_pass_radius"] ? items[i]["reverse_pass_radius"].as<double>() : reverse_pass_radius_;
          waypoints.push_back(w);
        } catch (const YAML::Exception & exc) {
          RCLCPP_WARN(get_logger(), "Skipping invalid waypoint #%zu: %s", i, exc.what());
        }
      }
    } catch (const YAML::Exception & exc) {
      RCLCPP_ERROR(get_logger(), "Failed to read waypoint YAML: %s", exc.what());
    }
    if (!waypoints.empty()) {
      selected_route_direction_ = resolved_direction;
    }
    return waypoints;
  }

  void control_once()
  {
    double robot_x = 0.0;
    double robot_y = 0.0;
    double robot_yaw = 0.0;
    if (!lookup_robot_pose(robot_x, robot_y, robot_yaw)) {
      publish_stop();
      return;
    }

    const auto & final = waypoints_.back();
    const double final_distance = std::hypot(final.x - robot_x, final.y - robot_y);
    if (current_index_ >= waypoints_.size() - 1 && final_distance <= goal_tolerance_) {
      RCLCPP_INFO(get_logger(), "Final waypoint reached; stopping.");
      publish_stop();
      rclcpp::shutdown();
      return;
    }

    advance_passed_waypoints(robot_x, robot_y);
    const std::size_t target_index = select_lookahead_waypoint(robot_x, robot_y);
    const auto & target = waypoints_[target_index];
    const auto tracking_target = select_tracking_target(robot_x, robot_y, target_index);
    const double active_speed = active_linear_speed(target_index);

    const double dx = tracking_target.x - robot_x;
    const double dy = tracking_target.y - robot_y;
    const double cos_yaw = std::cos(robot_yaw);
    const double sin_yaw = std::sin(robot_yaw);
    const double target_x_base = cos_yaw * dx + sin_yaw * dy;
    const double target_y_base = -sin_yaw * dx + cos_yaw * dy;
    const double distance = std::max(std::hypot(target_x_base, target_y_base), 0.001);

    double heading_error = std::atan2(target_y_base, target_x_base);
    double speed_scale = clamp_value(1.0 - std::abs(heading_error) / 1.8, turn_min_speed_scale_, 1.0);
    if (target_x_base < 0.0) {
      speed_scale = 0.25;
    }

    geometry_msgs::msg::Twist cmd;
    const double curvature = 2.0 * target_y_base / (distance * distance);
    if (tracking_target.reverse) {
      heading_error = std::atan2(-target_y_base, -target_x_base);
      speed_scale = clamp_value(1.0 - std::abs(heading_error) / 1.8, reverse_min_speed_scale_, 1.0);
      cmd.linear.x = -active_speed * speed_scale;
      cmd.angular.z = clamp_value(cmd.linear.x * curvature * reverse_angular_gain_, -max_angular_z_, max_angular_z_);
      cmd = apply_reverse_approach_nudge(cmd, target_index, distance);
    } else {
      cmd.linear.x = active_speed * speed_scale;
      cmd.angular.z = clamp_value(active_speed * curvature * turn_angular_gain_, -max_angular_z_, max_angular_z_);
    }

    cmd = apply_obstacle_avoidance(cmd, tracking_target.reverse);
    cmd_pub_->publish(cmd);
  }

  geometry_msgs::msg::Twist apply_reverse_approach_nudge(
    geometry_msgs::msg::Twist cmd, std::size_t target_index, double target_distance) const
  {
    const int waypoint_number = static_cast<int>(target_index) + 1;
    if (!enable_reverse_approach_nudge_ || waypoint_number != reverse_approach_nudge_waypoint_) {
      return cmd;
    }
    if (target_distance > reverse_approach_nudge_distance_) {
      return cmd;
    }

    cmd.angular.z = clamp_value(reverse_approach_nudge_angular_z_, -max_angular_z_, max_angular_z_);
    return cmd;
  }

  double active_linear_speed(std::size_t target_index)
  {
    const int waypoint_number = static_cast<int>(target_index) + 1;
    const bool in_channel = std::any_of(
      channel_waypoint_ranges_.begin(), channel_waypoint_ranges_.end(),
      [waypoint_number](const auto & range) { return range.first <= waypoint_number && waypoint_number <= range.second; });
    const std::string zone = in_channel ? "channel" : "outside";
    const double speed = in_channel ? channel_linear_speed_ : linear_speed_;
    if (zone != last_speed_zone_) {
      RCLCPP_INFO(get_logger(), "Speed zone: %s, waypoint %d, speed=%.2fm/s.", zone.c_str(), waypoint_number, speed);
      last_speed_zone_ = zone;
    }
    return speed;
  }

  bool lookup_robot_pose(double & x, double & y, double & yaw)
  {
    try {
      const auto transform = tf_buffer_.lookupTransform(frame_id_, base_frame_id_, tf2::TimePointZero);
      const auto & translation = transform.transform.translation;
      const auto & rotation = transform.transform.rotation;
      x = translation.x;
      y = translation.y;
      yaw = std::atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z));
      return true;
    } catch (const tf2::TransformException & exc) {
      RCLCPP_WARN(get_logger(), "Waiting for TF %s->%s: %s", frame_id_.c_str(), base_frame_id_.c_str(), exc.what());
      return false;
    }
  }

  void advance_passed_waypoints(double robot_x, double robot_y)
  {
    while (current_index_ < waypoints_.size() - 1) {
      const auto & waypoint = waypoints_[current_index_];
      const double distance = std::hypot(waypoint.x - robot_x, waypoint.y - robot_y);
      const double pass_radius = waypoint.reverse ?
        std::max(waypoint.reverse_pass_radius, reverse_goal_lookahead_distance_) : waypoint.pass_radius;
      if (distance > pass_radius) {
        if (should_skip_overshot_waypoint(robot_x, robot_y, distance)) {
          const auto & next = waypoints_[current_index_ + 1];
          const double next_distance = std::hypot(next.x - robot_x, next.y - robot_y);
          RCLCPP_WARN(
            get_logger(),
            "Skipping overshot waypoint %zu/%zu: current_distance=%.2fm, next_distance=%.2fm.",
            current_index_ + 1, waypoints_.size(), distance, next_distance);
          trigger_image_analysis_if_needed(current_index_ + 1);
          ++current_index_;
          continue;
        }
        break;
      }
      RCLCPP_INFO(get_logger(), "Passing waypoint %zu/%zu at distance %.2fm.", current_index_ + 1, waypoints_.size(), distance);
      trigger_image_analysis_if_needed(current_index_ + 1);
      ++current_index_;
    }
  }

  bool should_skip_overshot_waypoint(double robot_x, double robot_y, double current_distance) const
  {
    if (!enable_skip_overshot_waypoints_ || current_index_ + 1 >= waypoints_.size()) {
      return false;
    }
    const int waypoint_number = static_cast<int>(current_index_) + 1;
    if (waypoint_number < skip_overshot_min_waypoint_ || waypoint_number > skip_overshot_max_waypoint_) {
      return false;
    }
    if (current_distance > overshot_waypoint_max_distance_) {
      return false;
    }

    const auto & next = waypoints_[current_index_ + 1];
    const double next_distance = std::hypot(next.x - robot_x, next.y - robot_y);
    if (next_distance > overshot_next_waypoint_max_distance_) {
      return false;
    }
    return next_distance + overshot_waypoint_margin_ < current_distance;
  }

  void trigger_image_analysis_if_needed(std::size_t waypoint_number)
  {
    if (!enable_image_analysis_trigger_ || image_analysis_triggered_) {
      return;
    }
    if (waypoint_number != static_cast<std::size_t>(image_analysis_waypoint_)) {
      return;
    }

    std_msgs::msg::Int32 msg;
    msg.data = image_analysis_waypoint_;
    image_analysis_trigger_pub_->publish(msg);
    image_analysis_triggered_ = true;
    RCLCPP_INFO(
      get_logger(), "Reached waypoint %d; triggered Aliyun image analysis on %s.",
      image_analysis_waypoint_, image_analysis_trigger_topic_.c_str());
  }

  std::size_t select_lookahead_waypoint(double robot_x, double robot_y) const
  {
    if (current_index_ < waypoints_.size() && waypoints_[current_index_].reverse) {
      return current_index_;
    }

    std::size_t target_index = current_index_;
    for (std::size_t index = current_index_; index < waypoints_.size(); ++index) {
      const auto & waypoint = waypoints_[index];
      const double distance = std::hypot(waypoint.x - robot_x, waypoint.y - robot_y);
      double lookahead = lookahead_distance_;
      if (waypoint.reverse && index == current_index_) {
        lookahead = std::min(lookahead_distance_, reverse_goal_lookahead_distance_);
      }
      if (distance >= lookahead) {
        target_index = index;
        break;
      }
      target_index = index;
    }
    return target_index;
  }

  TrackingTarget select_tracking_target(double robot_x, double robot_y, std::size_t target_index) const
  {
    const auto & target = waypoints_[target_index];
    TrackingTarget result{target.x, target.y, target.reverse};
    if (!enable_smooth_keepout_path_ || target.reverse || waypoints_.size() < 4) {
      return result;
    }

    const int waypoint_number = static_cast<int>(target_index) + 1;
    if (waypoint_number < smooth_path_min_waypoint_ || waypoint_number > smooth_path_max_waypoint_) {
      return result;
    }

    double best_score = std::numeric_limits<double>::infinity();
    TrackingTarget best = result;
    const double desired_distance = std::max(lookahead_distance_, 0.20);
    const std::size_t max_segment =
      std::min<std::size_t>(waypoints_.size() - 2, static_cast<std::size_t>(smooth_path_max_waypoint_ - 1));

    for (std::size_t segment = current_index_; segment <= max_segment; ++segment) {
      for (int sample = 1; sample <= smooth_path_samples_per_segment_; ++sample) {
        const double t = static_cast<double>(sample) / static_cast<double>(smooth_path_samples_per_segment_);
        const auto point = catmull_rom_point(segment, t);
        const auto next_point = catmull_rom_point(segment, std::min(1.0, t + 0.05));
        const double distance = std::hypot(point.first - robot_x, point.second - robot_y);
        if (distance < desired_distance * 0.55) {
          continue;
        }

        const double tangent_x = next_point.first - point.first;
        const double tangent_y = next_point.second - point.second;
        const double tangent_norm = std::max(std::hypot(tangent_x, tangent_y), 0.001);
        const double normal_x = -tangent_y / tangent_norm;
        const double normal_y = tangent_x / tangent_norm;
        const double offsets[] = {0.0, smooth_path_keepout_clearance_ * 0.5, -smooth_path_keepout_clearance_ * 0.5};
        for (const double offset : offsets) {
          const double candidate_x = point.first + normal_x * offset;
          const double candidate_y = point.second + normal_y * offset;
          const double candidate_distance = std::hypot(candidate_x - robot_x, candidate_y - robot_y);
          const double clearance = keepout_clearance_distance(
            candidate_x, candidate_y, std::max(smooth_path_keepout_clearance_, 0.01));
          const double keepout_penalty = std::max(0.0, smooth_path_keepout_clearance_ - clearance);
          const double offset_penalty = std::abs(offset) * 0.25;
          const double score =
            std::abs(candidate_distance - desired_distance) +
            keepout_penalty * smooth_path_keepout_weight_ +
            offset_penalty;
          if (score < best_score) {
            best_score = score;
            best.x = candidate_x;
            best.y = candidate_y;
            best.reverse = false;
          }
        }
      }
    }

    return best;
  }

  std::pair<double, double> catmull_rom_point(std::size_t segment, double t) const
  {
    const auto p0 = waypoint_point(segment == 0 ? 0 : segment - 1);
    const auto p1 = waypoint_point(segment);
    const auto p2 = waypoint_point(std::min(segment + 1, waypoints_.size() - 1));
    const auto p3 = waypoint_point(std::min(segment + 2, waypoints_.size() - 1));
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double x = 0.5 * (
      2.0 * p1.first +
      (-p0.first + p2.first) * t +
      (2.0 * p0.first - 5.0 * p1.first + 4.0 * p2.first - p3.first) * t2 +
      (-p0.first + 3.0 * p1.first - 3.0 * p2.first + p3.first) * t3);
    const double y = 0.5 * (
      2.0 * p1.second +
      (-p0.second + p2.second) * t +
      (2.0 * p0.second - 5.0 * p1.second + 4.0 * p2.second - p3.second) * t2 +
      (-p0.second + 3.0 * p1.second - 3.0 * p2.second + p3.second) * t3);
    return {x, y};
  }

  std::pair<double, double> waypoint_point(std::size_t index) const
  {
    const auto & waypoint = waypoints_[std::min(index, waypoints_.size() - 1)];
    return {waypoint.x, waypoint.y};
  }

  geometry_msgs::msg::Twist apply_obstacle_avoidance(geometry_msgs::msg::Twist cmd, bool reverse)
  {
    if (!enable_obstacle_avoidance_) {
      return cmd;
    }
    if (static_cast<int>(current_index_) + 1 < obstacle_enable_from_waypoint_) {
      return cmd;
    }

    cmd = apply_keepout_avoidance(cmd, reverse);

    if (!latest_scan_) {
      return cmd;
    }

    const double now = this->now().seconds();
    double front_min = 0.0;
    double left_min = 0.0;
    double right_min = 0.0;
    if (reverse) {
      front_min = min_range(M_PI - front_angle_, M_PI + front_angle_);
      left_min = min_range(M_PI, M_PI + side_angle_);
      right_min = min_range(M_PI - side_angle_, M_PI);
    } else {
      front_min = min_range(-front_angle_, front_angle_);
      left_min = min_range(0.0, side_angle_);
      right_min = min_range(-side_angle_, 0.0);
    }

    if (!std::isfinite(front_min)) {
      emergency_start_time_ = std::numeric_limits<double>::quiet_NaN();
      backup_until_time_ = std::numeric_limits<double>::quiet_NaN();
      return cmd;
    }

    if (std::isfinite(backup_until_time_)) {
      if (now < backup_until_time_) {
        return backup_cmd();
      }
      backup_until_time_ = std::numeric_limits<double>::quiet_NaN();
      emergency_start_time_ = std::numeric_limits<double>::quiet_NaN();
    }

    if (front_min <= obstacle_stop_distance_) {
      const double avoid_direction = avoid_direction_from_ranges(left_min, right_min);
      if (backup_trigger_time_ <= 0.0 && rear_is_clear()) {
        backup_until_time_ = now + backup_duration_;
        backup_turn_direction_ = avoid_direction;
        log_obstacle_warning("Obstacle emergency at " + std::to_string(front_min) + "m; backing up.");
        return backup_cmd();
      }
      if (!std::isfinite(emergency_start_time_)) {
        emergency_start_time_ = now;
      } else if (now - emergency_start_time_ >= backup_trigger_time_) {
        if (rear_is_clear()) {
          backup_until_time_ = now + backup_duration_;
          backup_turn_direction_ = avoid_direction;
          log_obstacle_warning("Obstacle stuck at " + std::to_string(front_min) + "m; backing up.");
          return backup_cmd();
        }
        emergency_start_time_ = now;
      }
      cmd.linear.x = 0.0;
      cmd.angular.z = avoid_direction * avoid_angular_z_;
      log_obstacle_warning("Obstacle emergency stop: " + std::to_string(front_min) + "m.");
      return cmd;
    }

    emergency_start_time_ = std::numeric_limits<double>::quiet_NaN();

    if (front_min <= obstacle_slow_distance_) {
      const double slow_scale = clamp_value(
        (front_min - obstacle_stop_distance_) /
          std::max(obstacle_slow_distance_ - obstacle_stop_distance_, 0.01),
        0.20, 1.0);
      cmd.linear.x *= slow_scale;
    }

    if (front_min <= obstacle_avoid_distance_) {
      const double avoid_direction = avoid_direction_from_ranges(left_min, right_min);
      const double avoid_gain = clamp_value(
        (obstacle_avoid_distance_ - front_min) /
          std::max(obstacle_avoid_distance_ - obstacle_stop_distance_, 0.01),
        0.0, 1.0);
      cmd.angular.z = clamp_value(
        cmd.angular.z + avoid_direction * avoid_angular_z_ * avoid_gain,
        -max_angular_z_, max_angular_z_);
      log_obstacle_warning(
        "Obstacle avoid: front=" + std::to_string(front_min) + "m, left=" +
        std::to_string(left_min) + "m, right=" + std::to_string(right_min) + "m.");
    }
    return cmd;
  }

  geometry_msgs::msg::Twist apply_keepout_avoidance(geometry_msgs::msg::Twist cmd, bool reverse)
  {
    if (!enable_keepout_avoidance_ || !latest_keepout_mask_) {
      return cmd;
    }

    double robot_x = 0.0;
    double robot_y = 0.0;
    double robot_yaw = 0.0;
    if (!lookup_robot_pose(robot_x, robot_y, robot_yaw)) {
      return cmd;
    }

    const double direction = reverse ? -1.0 : 1.0;
    const double heading = normalize_angle(robot_yaw + (reverse ? M_PI : 0.0));
    const double front_min = keepout_distance_along(robot_x, robot_y, heading, keepout_slow_distance_);
    if (!std::isfinite(front_min)) {
      return cmd;
    }

    const double left_heading = normalize_angle(heading + M_PI / 2.0);
    const double right_heading = normalize_angle(heading - M_PI / 2.0);
    const bool left_blocked = keepout_cell_occupied(
      robot_x + std::cos(left_heading) * keepout_side_sample_distance_,
      robot_y + std::sin(left_heading) * keepout_side_sample_distance_);
    const bool right_blocked = keepout_cell_occupied(
      robot_x + std::cos(right_heading) * keepout_side_sample_distance_,
      robot_y + std::sin(right_heading) * keepout_side_sample_distance_);
    const double avoid_direction = right_blocked && !left_blocked ? 1.0 : -1.0;

    if (enable_keepout_hard_stop_ && front_min <= keepout_stop_distance_) {
      cmd.linear.x = 0.0;
      cmd.angular.z = avoid_direction * avoid_angular_z_ * direction;
      log_obstacle_warning("Keepout stop: virtual obstacle at " + std::to_string(front_min) + "m.");
      return cmd;
    }

    const double slow_scale = clamp_value(
      (front_min - keepout_stop_distance_) /
        std::max(keepout_slow_distance_ - keepout_stop_distance_, 0.01),
      keepout_min_speed_scale_, 1.0);
    cmd.linear.x *= slow_scale;

    if (front_min <= keepout_avoid_distance_) {
      const double avoid_gain = clamp_value(
        (keepout_avoid_distance_ - front_min) /
          std::max(keepout_avoid_distance_ - keepout_stop_distance_, 0.01),
        0.0, 1.0);
      cmd.angular.z = clamp_value(
        cmd.angular.z + avoid_direction * avoid_angular_z_ * avoid_gain * direction,
        -max_angular_z_, max_angular_z_);
      log_obstacle_warning("Keepout soft avoid: virtual obstacle at " + std::to_string(front_min) + "m.");
    }

    return cmd;
  }

  double keepout_distance_along(double origin_x, double origin_y, double heading, double max_distance) const
  {
    const double step = latest_keepout_mask_->info.resolution > 0.0 ?
      std::max(0.02, static_cast<double>(latest_keepout_mask_->info.resolution) * 0.5) : 0.03;
    for (double distance = 0.0; distance <= max_distance; distance += step) {
      const double x = origin_x + std::cos(heading) * distance;
      const double y = origin_y + std::sin(heading) * distance;
      if (keepout_cell_occupied(x, y)) {
        return distance;
      }
    }
    return std::numeric_limits<double>::infinity();
  }

  double keepout_clearance_distance(double world_x, double world_y, double max_distance) const
  {
    if (!latest_keepout_mask_ || max_distance <= 0.0) {
      return max_distance;
    }
    if (keepout_cell_occupied(world_x, world_y)) {
      return 0.0;
    }

    const auto & map = *latest_keepout_mask_;
    const double resolution = map.info.resolution > 0.0 ? map.info.resolution : 0.05;
    const double step = std::max(0.02, resolution);
    for (double radius = step; radius <= max_distance; radius += step) {
      const int samples = std::max(8, static_cast<int>(std::ceil(2.0 * M_PI * radius / step)));
      for (int i = 0; i < samples; ++i) {
        const double angle = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(samples);
        const double x = world_x + std::cos(angle) * radius;
        const double y = world_y + std::sin(angle) * radius;
        if (keepout_cell_occupied(x, y)) {
          return radius;
        }
      }
    }
    return max_distance;
  }

  bool keepout_cell_occupied(double world_x, double world_y) const
  {
    if (!latest_keepout_mask_) {
      return false;
    }
    const auto & map = *latest_keepout_mask_;
    const double origin_x = map.info.origin.position.x;
    const double origin_y = map.info.origin.position.y;
    const double resolution = map.info.resolution;
    if (resolution <= 0.0) {
      return false;
    }
    const int cell_x = static_cast<int>(std::floor((world_x - origin_x) / resolution));
    const int cell_y = static_cast<int>(std::floor((world_y - origin_y) / resolution));
    if (cell_x < 0 || cell_y < 0 ||
      cell_x >= static_cast<int>(map.info.width) || cell_y >= static_cast<int>(map.info.height))
    {
      return false;
    }
    const auto index = static_cast<std::size_t>(cell_y) * map.info.width + static_cast<std::size_t>(cell_x);
    if (index >= map.data.size()) {
      return false;
    }
    return map.data[index] >= keepout_occupied_threshold_;
  }

  geometry_msgs::msg::Twist backup_cmd() const
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = -std::abs(backup_speed_);
    cmd.angular.z = backup_turn_direction_ * backup_angular_z_;
    return cmd;
  }

  bool rear_is_clear()
  {
    const double rear_left = min_range(M_PI - front_angle_, M_PI);
    const double rear_right = min_range(-M_PI, -M_PI + front_angle_);
    const double rear_min = std::min(rear_left, rear_right);
    return !std::isfinite(rear_min) || rear_min > backup_stop_distance_;
  }

  double avoid_direction_from_ranges(double left_min, double right_min) const
  {
    if (!std::isfinite(left_min) && !std::isfinite(right_min)) {
      return 1.0;
    }
    if (!std::isfinite(left_min)) {
      return -1.0;
    }
    if (!std::isfinite(right_min)) {
      return 1.0;
    }
    return left_min >= right_min ? 1.0 : -1.0;
  }

  double min_range(double min_angle, double max_angle) const
  {
    if (!latest_scan_) {
      return std::numeric_limits<double>::infinity();
    }
    const auto & scan = *latest_scan_;
    double result = std::numeric_limits<double>::infinity();
    double angle = scan.angle_min;
    const double min_valid_range = std::max(static_cast<double>(scan.range_min), min_obstacle_range_);
    min_angle = normalize_angle(min_angle);
    max_angle = normalize_angle(max_angle);
    for (const auto value : scan.ranges) {
      const double scan_angle = normalize_angle(angle);
      const bool in_sector = min_angle <= max_angle ?
        (min_angle <= scan_angle && scan_angle <= max_angle) :
        (scan_angle >= min_angle || scan_angle <= max_angle);
      if (in_sector && std::isfinite(value) && min_valid_range <= value && value <= scan.range_max) {
        result = std::min(result, static_cast<double>(value));
      }
      angle += scan.angle_increment;
    }
    return result;
  }

  void log_obstacle_warning(const std::string & message)
  {
    ++obstacle_warning_count_;
    if (obstacle_warning_count_ % 10 == 1) {
      RCLCPP_WARN(get_logger(), "%s", message.c_str());
    }
  }

  std::string waypoints_file_;
  std::string frame_id_;
  std::string base_frame_id_;
  std::string qr_topic_;
  std::string route_direction_;
  std::string selected_route_direction_;
  int clockwise_qr_value_{1};
  int counterclockwise_qr_value_{2};
  double linear_speed_{0.35};
  double channel_linear_speed_{0.45};
  std::vector<std::pair<int, int>> channel_waypoint_ranges_;
  double lookahead_distance_{0.55};
  double pass_radius_{0.35};
  bool enable_smooth_keepout_path_{true};
  int smooth_path_min_waypoint_{4};
  int smooth_path_max_waypoint_{9};
  int smooth_path_samples_per_segment_{10};
  double smooth_path_keepout_clearance_{0.22};
  double smooth_path_keepout_weight_{0.9};
  bool enable_skip_overshot_waypoints_{true};
  double overshot_waypoint_margin_{0.05};
  double overshot_waypoint_max_distance_{0.80};
  double overshot_next_waypoint_max_distance_{0.45};
  int skip_overshot_min_waypoint_{5};
  int skip_overshot_max_waypoint_{9};
  double goal_tolerance_{0.25};
  double max_angular_z_{1.6};
  double turn_angular_gain_{1.4};
  double turn_min_speed_scale_{0.45};
  double reverse_angular_gain_{1.1};
  double reverse_min_speed_scale_{0.30};
  double reverse_goal_lookahead_distance_{0.25};
  double reverse_pass_radius_{0.25};
  bool enable_reverse_approach_nudge_{true};
  int reverse_approach_nudge_waypoint_{2};
  double reverse_approach_nudge_distance_{0.35};
  double reverse_approach_nudge_angular_z_{-0.35};
  bool enable_obstacle_avoidance_{true};
  int obstacle_enable_from_waypoint_{0};
  double obstacle_stop_distance_{0.22};
  double obstacle_slow_distance_{0.80};
  double obstacle_avoid_distance_{0.60};
  double min_obstacle_range_{0.10};
  double front_angle_{M_PI / 6.0};
  double side_angle_{M_PI / 2.0};
  double avoid_angular_z_{0.55};
  bool enable_keepout_avoidance_{true};
  int keepout_occupied_threshold_{50};
  double keepout_stop_distance_{0.25};
  double keepout_slow_distance_{0.70};
  double keepout_avoid_distance_{0.55};
  double keepout_side_sample_distance_{0.35};
  bool enable_keepout_hard_stop_{false};
  double keepout_min_speed_scale_{0.45};
  double backup_trigger_time_{0.0};
  double backup_duration_{0.5};
  double backup_speed_{0.12};
  double backup_angular_z_{0.35};
  double backup_stop_distance_{0.18};
  bool enable_qr_skip_first_{true};
  bool enable_qr_direction_select_{true};
  bool use_qr_parity_direction_{true};
  bool enable_image_analysis_trigger_{true};
  int image_analysis_waypoint_{7};
  std::string image_analysis_trigger_topic_{"/person_trigger"};

  std::vector<Waypoint> waypoints_;
  std::size_t current_index_{0};
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_keepout_mask_;
  int obstacle_warning_count_{0};
  double emergency_start_time_{std::numeric_limits<double>::quiet_NaN()};
  double backup_until_time_{std::numeric_limits<double>::quiet_NaN()};
  double backup_turn_direction_{1.0};
  bool qr_skip_done_{false};
  bool image_analysis_triggered_{false};
  std::string last_speed_zone_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr image_analysis_trigger_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr keepout_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr qr_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SmoothPathFollower>();
  rclcpp::spin(node);
  node->publish_stop();
  rclcpp::shutdown();
  return 0;
}
