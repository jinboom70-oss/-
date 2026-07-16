#include <algorithm>
#include <chrono>
#include <cctype>
#include <memory>
#include <string>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <zbar.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/string.hpp"

class QRDetectorNode : public rclcpp::Node
{
public:
  QRDetectorNode() : Node("qr_detector_node")
  {
    declare_parameter<std::string>("image_topic", "/image");
    declare_parameter<std::string>("qr_topic", "/qr_info");
    declare_parameter<double>("scan_hz", 30.0);
    declare_parameter<int>("resize_width", 0);
    declare_parameter<double>("repeat_publish_interval", 0.3);

    image_topic_ = get_parameter("image_topic").as_string();
    qr_topic_ = get_parameter("qr_topic").as_string();
    scan_hz_ = std::max(1.0, get_parameter("scan_hz").as_double());
    resize_width_ = static_cast<int>(get_parameter("resize_width").as_int());
    repeat_publish_interval_ = std::max(0.0, get_parameter("repeat_publish_interval").as_double());

    scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
    scanner_.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);

    image_subscription_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      image_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&QRDetectorNode::image_callback, this, std::placeholders::_1));

    qr_info_publisher_ = create_publisher<std_msgs::msg::String>(qr_topic_, 10);

    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / scan_hz_));

    scan_timer_ = create_wall_timer(
      period,
      std::bind(&QRDetectorNode::scan_latest_image, this));

    RCLCPP_INFO(get_logger(), "QR Detector Node started with ZBar.");
  }

private:
  void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    latest_msg_ = msg;
    latest_stamp_sec_ = msg->header.stamp.sec;
    latest_stamp_nanosec_ = msg->header.stamp.nanosec;
  }

  void scan_latest_image()
  {
    if (!latest_msg_) {
      return;
    }

    if (latest_stamp_sec_ == last_processed_stamp_sec_ &&
        latest_stamp_nanosec_ == last_processed_stamp_nanosec_) {
      return;
    }

    last_processed_stamp_sec_ = latest_stamp_sec_;
    last_processed_stamp_nanosec_ = latest_stamp_nanosec_;

    cv::Mat gray = cv::imdecode(latest_msg_->data, cv::IMREAD_GRAYSCALE);
    if (gray.empty()) {
      return;
    }

    if (resize_width_ > 0 && gray.cols > resize_width_) {
      double scale = static_cast<double>(resize_width_) / static_cast<double>(gray.cols);
      int height = std::max(1, static_cast<int>(gray.rows * scale));
      cv::resize(gray, gray, cv::Size(resize_width_, height), 0.0, 0.0, cv::INTER_AREA);
    }

    if (!gray.isContinuous()) {
      gray = gray.clone();
    }

    zbar::Image image(
      gray.cols,
      gray.rows,
      "Y800",
      gray.data,
      static_cast<unsigned long>(gray.total()));

    int count = scanner_.scan(image);

    if (count > 0) {
      for (auto symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
        std::string data = symbol->get_data();
        trim(data);

        if (is_valid_number(data)) {
          publish_qr(data);
          break;
        }
      }
    }

    image.set_data(nullptr, 0);
  }

  static void trim(std::string & text)
  {
    text.erase(
      text.begin(),
      std::find_if(text.begin(), text.end(), [](unsigned char ch) {
        return !std::isspace(ch);
      }));

    text.erase(
      std::find_if(text.rbegin(), text.rend(), [](unsigned char ch) {
        return !std::isspace(ch);
      }).base(),
      text.end());
  }

  bool is_valid_number(const std::string & text)
  {
    if (text.empty()) {
      return false;
    }

    for (char c : text) {
      if (!std::isdigit(static_cast<unsigned char>(c))) {
        return false;
      }
    }

    int value = std::stoi(text);
    return value >= 0 && value <= 9999;
  }

  void publish_qr(const std::string & data)
  {
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_publish_time_).count();

    if (data == last_data_ && dt < repeat_publish_interval_) {
      return;
    }

    std_msgs::msg::String msg;
    msg.data = data;
    qr_info_publisher_->publish(msg);

    last_data_ = data;
    last_publish_time_ = now;

    RCLCPP_INFO(get_logger(), "Detected valid QR Code number: %s", data.c_str());
  }

private:
  std::string image_topic_;
  std::string qr_topic_;
  double scan_hz_;
  int resize_width_;
  double repeat_publish_interval_;

  sensor_msgs::msg::CompressedImage::SharedPtr latest_msg_;

  int32_t latest_stamp_sec_{0};
  uint32_t latest_stamp_nanosec_{0};
  int32_t last_processed_stamp_sec_{-1};
  uint32_t last_processed_stamp_nanosec_{0};

  std::string last_data_;
  std::chrono::steady_clock::time_point last_publish_time_{
    std::chrono::steady_clock::time_point::min()};

  zbar::ImageScanner scanner_;

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr qr_info_publisher_;
  rclcpp::TimerBase::SharedPtr scan_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QRDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
