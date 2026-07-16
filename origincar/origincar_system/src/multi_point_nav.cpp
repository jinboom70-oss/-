#include <algorithm>
#include <cctype>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "action_msgs/msg/goal_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{

bool as_bool(const rclcpp::Parameter & parameter)
{
  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
    return parameter.as_bool();
  }
  const auto text = parameter.value_to_string();
  return text == "1" || text == "true" || text == "True" || text == "TRUE" ||
         text == "yes" || text == "on" || text == "ON";
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

double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

std::string to_lower(std::string text)
{
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return text;
}

struct WaypointOptions
{
  double pass_radius{0.0};
};

}  // namespace

class MultiPointNavigator : public rclcpp::Node
{
public:
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using FollowGoalHandle = rclcpp_action::ClientGoalHandle<FollowWaypoints>;
  using ThroughGoalHandle = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;
  using PoseGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  MultiPointNavigator()
  : Node("multi_point_navigator"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameter<std::string>("waypoints_file", "");
    declare_parameter<std::string>("action_type", "ordered_smooth");
    declare_parameter<std::string>("action_name", "");
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<std::string>("base_frame_id", "base_link");
    declare_parameter<double>("start_delay", 0.0);
    declare_parameter<double>("retry_delay", 3.0);
    declare_parameter<int>("max_goal_retries", 2);
    declare_parameter<bool>("skip_failed_waypoints", true);
    declare_parameter<double>("pass_radius", 0.6);
    declare_parameter<double>("monitor_period", 0.2);
    declare_parameter<bool>("auto_segment_yaw", true);
    declare_parameter<std::string>("route_direction", "auto");
    declare_parameter<bool>("enable_qr_direction_select", true);
    declare_parameter<std::string>("qr_topic", "/qr_info");
    declare_parameter<bool>("use_qr_parity_direction", true);
    declare_parameter<int>("clockwise_qr_value", 1);
    declare_parameter<int>("counterclockwise_qr_value", 2);
    declare_parameter<bool>("enable_image_analysis_trigger", true);
    declare_parameter<int>("image_analysis_waypoint", 7);
    declare_parameter<std::string>("image_analysis_trigger_topic", "/person_trigger");

    waypoints_file_ = get_parameter("waypoints_file").as_string();
    action_type_ = get_parameter("action_type").as_string();
    action_name_ = get_parameter("action_name").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    base_frame_id_ = get_parameter("base_frame_id").as_string();
    start_delay_ = get_parameter("start_delay").as_double();
    retry_delay_ = get_parameter("retry_delay").as_double();
    max_goal_retries_ = get_parameter("max_goal_retries").as_int();
    skip_failed_waypoints_ = as_bool(get_parameter("skip_failed_waypoints"));
    pass_radius_ = get_parameter("pass_radius").as_double();
    monitor_period_ = get_parameter("monitor_period").as_double();
    auto_segment_yaw_ = as_bool(get_parameter("auto_segment_yaw"));
    route_direction_ = to_lower(get_parameter("route_direction").as_string());
    enable_qr_direction_select_ = as_bool(get_parameter("enable_qr_direction_select"));
    qr_topic_ = get_parameter("qr_topic").as_string();
    use_qr_parity_direction_ = as_bool(get_parameter("use_qr_parity_direction"));
    clockwise_qr_value_ = static_cast<int>(get_parameter("clockwise_qr_value").as_int());
    counterclockwise_qr_value_ = static_cast<int>(get_parameter("counterclockwise_qr_value").as_int());
    enable_image_analysis_trigger_ = as_bool(get_parameter("enable_image_analysis_trigger"));
    image_analysis_waypoint_ = std::max(1, static_cast<int>(get_parameter("image_analysis_waypoint").as_int()));
    image_analysis_trigger_topic_ = get_parameter("image_analysis_trigger_topic").as_string();

    if (action_type_ == "ordered_smooth") {
      action_name_ = action_name_.empty() ? "/navigate_to_pose" : action_name_;
      nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, action_name_);
    } else if (action_type_ == "follow_waypoints") {
      action_name_ = action_name_.empty() ? "/follow_waypoints" : action_name_;
      follow_waypoints_client_ = rclcpp_action::create_client<FollowWaypoints>(this, action_name_);
    } else {
      action_type_ = "navigate_through_poses";
      action_name_ = action_name_.empty() ? "/navigate_through_poses" : action_name_;
      navigate_through_poses_client_ = rclcpp_action::create_client<NavigateThroughPoses>(this, action_name_);
    }

    image_analysis_trigger_pub_ =
      create_publisher<std_msgs::msg::Int32>(image_analysis_trigger_topic_, rclcpp::QoS(10).transient_local());
    if (enable_qr_direction_select_) {
      qr_sub_ = create_subscription<std_msgs::msg::String>(
        qr_topic_, 10, std::bind(&MultiPointNavigator::qr_callback, this, std::placeholders::_1));
    }

    timer_ = create_wall_timer(
      std::chrono::duration<double>(std::max(start_delay_, 0.1)),
      std::bind(&MultiPointNavigator::start_once, this));
  }

private:
  void start_once()
  {
    timer_->cancel();
    poses_ = load_waypoints(route_direction_);
    if (poses_.empty()) {
      RCLCPP_ERROR(get_logger(), "No waypoints loaded; nothing to send.");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(get_logger(), "Waiting for Nav2 action: %s (%s)", action_name_.c_str(), action_type_.c_str());
    const auto timeout = std::chrono::seconds(30);
    bool server_ready = false;
    if (action_type_ == "ordered_smooth") {
      server_ready = nav_to_pose_client_->wait_for_action_server(timeout);
    } else if (action_type_ == "follow_waypoints") {
      server_ready = follow_waypoints_client_->wait_for_action_server(timeout);
    } else {
      server_ready = navigate_through_poses_client_->wait_for_action_server(timeout);
    }
    if (!server_ready) {
      RCLCPP_ERROR(get_logger(), "Action server not available: %s", action_name_.c_str());
      rclcpp::shutdown();
      return;
    }

    send_goal();
  }

  void qr_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string direction = direction_from_qr(msg->data);
    if (direction.empty() || direction == selected_route_direction_) {
      return;
    }

    if (action_type_ != "ordered_smooth") {
      RCLCPP_WARN(
        get_logger(), "QR selected %s route, but action_type=%s already sends the whole route; use ordered_smooth for live route switching.",
        direction.c_str(), action_type_.c_str());
      return;
    }

    const auto new_poses = load_waypoints(direction);
    if (new_poses.empty()) {
      RCLCPP_WARN(get_logger(), "QR selected route \"%s\", but no waypoints were loaded.", direction.c_str());
      return;
    }

    poses_ = new_poses;
    current_pose_index_ = std::min(current_pose_index_, poses_.size() - 1);
    goal_retry_count_ = 0;
    RCLCPP_INFO(
      get_logger(), "QR selected %s route; continuing from waypoint %zu/%zu.",
      selected_route_direction_.c_str(), current_pose_index_ + 1, poses_.size());
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

  std::vector<geometry_msgs::msg::PoseStamped> load_waypoints(const std::string & requested_direction)
  {
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    pose_modes_.clear();
    waypoint_options_.clear();
    std::string resolved_direction = requested_direction;
    if (waypoints_file_.empty()) {
      RCLCPP_ERROR(get_logger(), "Parameter waypoints_file is empty.");
      return poses;
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
        return poses;
      }

      for (std::size_t index = 0; index < items.size(); ++index) {
        try {
          const auto item = items[index];
          const double x = item["x"].as<double>();
          const double y = item["y"].as<double>();
          const double z = item["z"] ? item["z"].as<double>() : 0.0;
          const double yaw = waypoint_yaw(items, index, item);

          geometry_msgs::msg::PoseStamped pose;
          pose.header.frame_id = item["frame_id"] ? item["frame_id"].as<std::string>() : frame_id_;
          pose.header.stamp = now();
          pose.pose.position.x = x;
          pose.pose.position.y = y;
          pose.pose.position.z = z;
          pose.pose.orientation.z = std::sin(yaw * 0.5);
          pose.pose.orientation.w = std::cos(yaw * 0.5);
          poses.push_back(pose);
          pose_modes_.push_back(index > 0 && yaml_bool(item["reverse"], false) ? "reverse" : "forward");
          WaypointOptions options;
          options.pass_radius = item["pass_radius"] ? item["pass_radius"].as<double>() : pass_radius_;
          waypoint_options_.push_back(options);
        } catch (const YAML::Exception & exc) {
          RCLCPP_WARN(get_logger(), "Skipping invalid waypoint #%zu: %s", index, exc.what());
        }
      }
    } catch (const YAML::Exception & exc) {
      RCLCPP_ERROR(get_logger(), "Failed to read waypoint YAML: %s", exc.what());
    }

    if (!poses.empty()) {
      selected_route_direction_ = resolved_direction;
    }
    return poses;
  }

  double waypoint_yaw(const YAML::Node & items, std::size_t index, const YAML::Node & item) const
  {
    if (!auto_segment_yaw_ || index == 0) {
      return item["yaw"] ? item["yaw"].as<double>() : 0.0;
    }

    const auto previous = items[index - 1];
    const double dx = item["x"].as<double>() - previous["x"].as<double>();
    const double dy = item["y"].as<double>() - previous["y"].as<double>();
    if (std::abs(dx) < 1e-6 && std::abs(dy) < 1e-6) {
      return item["yaw"] ? item["yaw"].as<double>() : 0.0;
    }

    const double travel_yaw = std::atan2(dy, dx);
    if (yaml_bool(item["reverse"], false)) {
      return normalize_angle(travel_yaw + M_PI);
    }
    return travel_yaw;
  }

  void send_goal()
  {
    if (action_type_ == "ordered_smooth") {
      send_ordered_smooth_goal();
    } else if (action_type_ == "follow_waypoints") {
      send_follow_waypoints_goal();
    } else {
      send_navigate_through_poses_goal();
    }
  }

  void send_ordered_smooth_goal()
  {
    NavigateToPose::Goal goal;
    const std::size_t goal_index = current_pose_index_;
    const bool is_final_goal = goal_index == poses_.size() - 1;
    goal.pose = poses_[goal_index];
    active_goal_index_ = static_cast<int>(goal_index);
    final_goal_sent_ = is_final_goal;

    RCLCPP_INFO(
      get_logger(), "Sending pose %zu/%zu (%s).", goal_index + 1, poses_.size(),
      pose_modes_[goal_index].c_str());

    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    options.feedback_callback =
      [this](PoseGoalHandle::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        if (final_goal_sent_) {
          RCLCPP_INFO(get_logger(), "Final goal distance remaining: %.2f m", feedback->distance_remaining);
        }
      };
    options.goal_response_callback =
      [this, goal_index, is_final_goal](PoseGoalHandle::SharedPtr goal_handle) {
        handle_pose_goal_response(goal_handle, goal_index, is_final_goal);
      };
    options.result_callback =
      [this, goal_index, is_final_goal](const PoseGoalHandle::WrappedResult & result) {
        handle_pose_result(result, goal_index, is_final_goal);
      };
    nav_to_pose_client_->async_send_goal(goal, options);
  }

  void send_follow_waypoints_goal()
  {
    FollowWaypoints::Goal goal;
    goal.poses = poses_;
    RCLCPP_INFO(get_logger(), "Sending %zu pose(s).", poses_.size());

    auto options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
    options.feedback_callback =
      [this](FollowGoalHandle::SharedPtr, const std::shared_ptr<const FollowWaypoints::Feedback> feedback) {
        RCLCPP_INFO(get_logger(), "Current waypoint index: %u", feedback->current_waypoint);
      };
    options.goal_response_callback =
      [this](FollowGoalHandle::SharedPtr goal_handle) { handle_generic_goal_response(static_cast<bool>(goal_handle)); };
    options.result_callback =
      [this](const FollowGoalHandle::WrappedResult & result) {
        const auto missed = result.result ? result.result->missed_waypoints : std::vector<int32_t>{};
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED && missed.empty()) {
          RCLCPP_INFO(get_logger(), "All waypoints completed.");
        } else {
          RCLCPP_WARN(get_logger(), "Waypoint navigation finished with result code=%d, missed_waypoints=%zu", static_cast<int>(result.code), missed.size());
        }
        rclcpp::shutdown();
      };
    follow_waypoints_client_->async_send_goal(goal, options);
  }

  void send_navigate_through_poses_goal()
  {
    NavigateThroughPoses::Goal goal;
    goal.poses = poses_;
    RCLCPP_INFO(get_logger(), "Sending %zu pose(s).", poses_.size());

    auto options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
    options.feedback_callback =
      [this](ThroughGoalHandle::SharedPtr, const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback) {
        RCLCPP_INFO(
          get_logger(), "Poses remaining: %u, distance remaining: %.2f m",
          feedback->number_of_poses_remaining, feedback->distance_remaining);
      };
    options.goal_response_callback =
      [this](ThroughGoalHandle::SharedPtr goal_handle) { handle_generic_goal_response(static_cast<bool>(goal_handle)); };
    options.result_callback =
      [this](const ThroughGoalHandle::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(get_logger(), "Multi-point route completed.");
        } else {
          RCLCPP_WARN(get_logger(), "Multi-point route finished with result code=%d.", static_cast<int>(result.code));
        }
        rclcpp::shutdown();
      };
    navigate_through_poses_client_->async_send_goal(goal, options);
  }

  void handle_generic_goal_response(bool accepted)
  {
    if (!accepted) {
      RCLCPP_ERROR(get_logger(), "Navigation goal was rejected.");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(get_logger(), "Navigation goal accepted.");
  }

  void handle_pose_goal_response(PoseGoalHandle::SharedPtr goal_handle, std::size_t goal_index, bool is_final_goal)
  {
    if (!goal_handle) {
      if (goal_retry_count_ < max_goal_retries_) {
        ++goal_retry_count_;
        RCLCPP_WARN(
          get_logger(), "Navigation goal was rejected; retrying %d/%d in %.1fs.",
          goal_retry_count_, max_goal_retries_, retry_delay_);
        retry_timer_ = create_wall_timer(
          std::chrono::duration<double>(retry_delay_), std::bind(&MultiPointNavigator::retry_goal_once, this));
      } else {
        RCLCPP_ERROR(get_logger(), "Navigation goal was rejected; max retries reached.");
        rclcpp::shutdown();
      }
      return;
    }

    RCLCPP_INFO(get_logger(), "Navigation goal accepted.");
    if (action_type_ == "ordered_smooth" && !is_final_goal) {
      start_monitoring();
    }
    (void)goal_index;
  }

  void start_monitoring()
  {
    if (!monitor_timer_) {
      monitor_timer_ = create_wall_timer(
        std::chrono::duration<double>(monitor_period_), std::bind(&MultiPointNavigator::monitor_waypoint, this));
    }
  }

  void monitor_waypoint()
  {
    if (final_goal_sent_ || poses_.empty()) {
      return;
    }

    try {
      const auto transform = tf_buffer_.lookupTransform(frame_id_, base_frame_id_, tf2::TimePointZero);
      const auto & current_pose = poses_[current_pose_index_].pose.position;
      const auto & robot = transform.transform.translation;
      const double distance = std::hypot(current_pose.x - robot.x, current_pose.y - robot.y);
      const double waypoint_pass_radius = current_pose_index_ < waypoint_options_.size() ?
        waypoint_options_[current_pose_index_].pass_radius : pass_radius_;
      if (distance > waypoint_pass_radius) {
        return;
      }
      RCLCPP_INFO(
        get_logger(), "Passing waypoint %zu/%zu at distance %.2f m; switching to next.",
        current_pose_index_ + 1, poses_.size(), distance);
      trigger_image_analysis_if_needed(current_pose_index_ + 1);
      ++current_pose_index_;
      goal_retry_count_ = 0;
      if (current_pose_index_ < poses_.size()) {
        send_goal();
      }
    } catch (const tf2::TransformException & exc) {
      RCLCPP_WARN(get_logger(), "Waiting for TF %s->%s: %s", frame_id_.c_str(), base_frame_id_.c_str(), exc.what());
    }
  }

  void retry_goal_once()
  {
    if (retry_timer_) {
      retry_timer_->cancel();
      retry_timer_.reset();
    }
    send_goal();
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

  void handle_pose_result(const PoseGoalHandle::WrappedResult & result, std::size_t goal_index, bool is_final_goal)
  {
    if (!is_final_goal && goal_index < current_pose_index_) {
      RCLCPP_INFO(get_logger(), "Pose %zu/%zu was preempted by the next pose.", goal_index + 1, poses_.size());
      return;
    }

    if (!is_final_goal) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(get_logger(), "Pose %zu/%zu reached by Nav2.", goal_index + 1, poses_.size());
        trigger_image_analysis_if_needed(goal_index + 1);
        if (goal_index == current_pose_index_) {
          ++current_pose_index_;
          goal_retry_count_ = 0;
          send_goal();
        }
        return;
      }

      if (goal_retry_count_ < max_goal_retries_) {
        ++goal_retry_count_;
        RCLCPP_WARN(
          get_logger(), "Pose %zu/%zu failed with result code=%d; retrying %d/%d in %.1fs.",
          goal_index + 1, poses_.size(), static_cast<int>(result.code), goal_retry_count_,
          max_goal_retries_, retry_delay_);
        retry_timer_ = create_wall_timer(
          std::chrono::duration<double>(retry_delay_), std::bind(&MultiPointNavigator::retry_goal_once, this));
        return;
      }

      RCLCPP_ERROR(get_logger(), "Pose %zu/%zu failed; max retries reached.", goal_index + 1, poses_.size());
      if (skip_failed_waypoints_) {
        ++current_pose_index_;
        goal_retry_count_ = 0;
        if (current_pose_index_ < poses_.size()) {
          RCLCPP_WARN(get_logger(), "Skipping pose %zu/%zu and continuing.", goal_index + 1, poses_.size());
          send_goal();
        } else {
          rclcpp::shutdown();
        }
      } else {
        rclcpp::shutdown();
      }
      return;
    }

    if (goal_index < current_pose_index_) {
      RCLCPP_INFO(get_logger(), "Final pose %zu/%zu was preempted.", goal_index + 1, poses_.size());
      return;
    }

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(get_logger(), "Ordered smooth route completed.");
    } else {
      RCLCPP_WARN(get_logger(), "Ordered smooth route finished with result code=%d.", static_cast<int>(result.code));
    }
    rclcpp::shutdown();
  }

  std::string waypoints_file_;
  std::string action_type_;
  std::string action_name_;
  std::string frame_id_;
  std::string base_frame_id_;
  std::string route_direction_;
  std::string selected_route_direction_;
  std::string qr_topic_;
  int clockwise_qr_value_{1};
  int counterclockwise_qr_value_{2};
  double start_delay_{0.0};
  double retry_delay_{3.0};
  int max_goal_retries_{2};
  bool skip_failed_waypoints_{true};
  double pass_radius_{0.6};
  double monitor_period_{0.2};
  bool auto_segment_yaw_{true};
  bool enable_qr_direction_select_{true};
  bool use_qr_parity_direction_{true};
  bool enable_image_analysis_trigger_{true};
  int image_analysis_waypoint_{7};
  std::string image_analysis_trigger_topic_{"/person_trigger"};

  std::vector<geometry_msgs::msg::PoseStamped> poses_;
  std::vector<std::string> pose_modes_;
  std::vector<WaypointOptions> waypoint_options_;
  int goal_retry_count_{0};
  std::size_t current_pose_index_{0};
  int active_goal_index_{-1};
  bool final_goal_sent_{false};
  bool image_analysis_triggered_{false};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr image_analysis_trigger_pub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
  rclcpp_action::Client<FollowWaypoints>::SharedPtr follow_waypoints_client_;
  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr navigate_through_poses_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr qr_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiPointNavigator>());
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
