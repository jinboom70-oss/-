#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <curl/curl.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{

std::string json_escape(const std::string & text)
{
  std::ostringstream out;
  for (const unsigned char ch : text) {
    switch (ch) {
      case '"':
        out << "\\\"";
        break;
      case '\\':
        out << "\\\\";
        break;
      case '\b':
        out << "\\b";
        break;
      case '\f':
        out << "\\f";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        if (ch < 0x20) {
          out << "\\u00";
          const char hex[] = "0123456789abcdef";
          out << hex[(ch >> 4) & 0x0f] << hex[ch & 0x0f];
        } else {
          out << static_cast<char>(ch);
        }
        break;
    }
  }
  return out.str();
}

std::string base64_encode(const std::vector<unsigned char> & data)
{
  static constexpr char table[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string encoded;
  encoded.reserve(((data.size() + 2) / 3) * 4);

  for (std::size_t i = 0; i < data.size(); i += 3) {
    const unsigned int b0 = data[i];
    const unsigned int b1 = i + 1 < data.size() ? data[i + 1] : 0;
    const unsigned int b2 = i + 2 < data.size() ? data[i + 2] : 0;
    const unsigned int triple = (b0 << 16) | (b1 << 8) | b2;

    encoded.push_back(table[(triple >> 18) & 0x3f]);
    encoded.push_back(table[(triple >> 12) & 0x3f]);
    encoded.push_back(i + 1 < data.size() ? table[(triple >> 6) & 0x3f] : '=');
    encoded.push_back(i + 2 < data.size() ? table[triple & 0x3f] : '=');
  }

  return encoded;
}

std::string unescape_json_string(const std::string & text)
{
  std::string out;
  out.reserve(text.size());
  for (std::size_t i = 0; i < text.size(); ++i) {
    if (text[i] != '\\' || i + 1 >= text.size()) {
      out.push_back(text[i]);
      continue;
    }
    const char esc = text[++i];
    switch (esc) {
      case '"':
      case '\\':
      case '/':
        out.push_back(esc);
        break;
      case 'b':
        out.push_back('\b');
        break;
      case 'f':
        out.push_back('\f');
        break;
      case 'n':
        out.push_back('\n');
        break;
      case 'r':
        out.push_back('\r');
        break;
      case 't':
        out.push_back('\t');
        break;
      default:
        out.push_back(esc);
        break;
    }
  }
  return out;
}

std::string extract_json_string_value(const std::string & body, const std::string & key)
{
  const std::string marker = "\"" + key + "\"";
  const std::size_t key_pos = body.find(marker);
  if (key_pos == std::string::npos) {
    return "";
  }
  const std::size_t colon = body.find(':', key_pos + marker.size());
  if (colon == std::string::npos) {
    return "";
  }
  std::size_t quote = body.find('"', colon + 1);
  if (quote == std::string::npos) {
    return "";
  }

  std::string value;
  bool escaped = false;
  for (++quote; quote < body.size(); ++quote) {
    const char ch = body[quote];
    if (escaped) {
      value.push_back('\\');
      value.push_back(ch);
      escaped = false;
      continue;
    }
    if (ch == '\\') {
      escaped = true;
      continue;
    }
    if (ch == '"') {
      return unescape_json_string(value);
    }
    value.push_back(ch);
  }
  return "";
}

std::string extract_dashscope_content(const std::string & body)
{
  const std::string content = extract_json_string_value(body, "content");
  if (!content.empty()) {
    return content;
  }
  throw std::runtime_error("Failed to parse DashScope response: " + body);
}

std::string trim(std::string text)
{
  text.erase(text.begin(), std::find_if(text.begin(), text.end(), [](unsigned char ch) {
    return !std::isspace(ch);
  }));
  text.erase(std::find_if(text.rbegin(), text.rend(), [](unsigned char ch) {
    return !std::isspace(ch);
  }).base(), text.end());
  return text;
}

std::size_t dashscope_write_callback(char * ptr, std::size_t size, std::size_t nmemb, void * userdata)
{
  auto * body = static_cast<std::string *>(userdata);
  body->append(ptr, size * nmemb);
  return size * nmemb;
}

}  // namespace

class AliyunImageAnalyzer : public rclcpp::Node
{
public:
  AliyunImageAnalyzer()
  : Node("aliyun_image_analyzer")
  {
    declare_parameter<std::string>("image_topic", "/image");
    declare_parameter<std::string>("image_msg_type", "compressed");
    declare_parameter<std::string>("trigger_topic", "/person_trigger");
    declare_parameter<std::string>("trigger_msg_type", "bool");
    declare_parameter<int>("trigger_value", 30);
    declare_parameter<std::string>("result_topic", "/image_ai");
    declare_parameter<std::string>("api_key", "");
    declare_parameter<std::string>("api_key_env", "DASHSCOPE_API_KEY");
    declare_parameter<std::string>(
      "base_url", "https://dashscope.aliyuncs.com/compatible-mode/v1/chat/completions");
    declare_parameter<std::string>("model", "qwen-vl-plus");
    declare_parameter<std::string>("prompt", "请识别图像中的关键内容，只输出最终识别结果。");
    declare_parameter<std::string>("image_mime", "image/jpeg");
    declare_parameter<double>("request_timeout_sec", 20.0);
    declare_parameter<double>("min_interval_sec", 3.0);
    declare_parameter<std::string>("publish_prefix", "图像分析结果：");
    declare_parameter<bool>("save_debug_image", false);
    declare_parameter<std::string>("debug_image_dir", "/tmp/smart_medical_images");

    image_topic_ = get_parameter("image_topic").as_string();
    image_msg_type_ = get_parameter("image_msg_type").as_string();
    trigger_topic_ = get_parameter("trigger_topic").as_string();
    trigger_msg_type_ = get_parameter("trigger_msg_type").as_string();
    trigger_value_ = static_cast<int>(get_parameter("trigger_value").as_int());
    result_topic_ = get_parameter("result_topic").as_string();
    api_key_ = get_parameter("api_key").as_string();
    api_key_env_ = get_parameter("api_key_env").as_string();
    base_url_ = get_parameter("base_url").as_string();
    model_ = get_parameter("model").as_string();
    prompt_ = get_parameter("prompt").as_string();
    image_mime_ = get_parameter("image_mime").as_string();
    request_timeout_sec_ = get_parameter("request_timeout_sec").as_double();
    min_interval_sec_ = get_parameter("min_interval_sec").as_double();
    publish_prefix_ = get_parameter("publish_prefix").as_string();
    save_debug_image_ = get_parameter("save_debug_image").as_bool();
    debug_image_dir_ = get_parameter("debug_image_dir").as_string();

    result_pub_ = create_publisher<std_msgs::msg::String>(result_topic_, 10);

    if (image_msg_type_ == "raw" || image_msg_type_ == "image" ||
      image_msg_type_ == "sensor_msgs/image")
    {
      raw_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic_, 10, std::bind(&AliyunImageAnalyzer::on_raw_image, this, std::placeholders::_1));
    } else {
      compressed_image_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
        image_topic_, 10,
        std::bind(&AliyunImageAnalyzer::on_compressed_image, this, std::placeholders::_1));
    }

    if (trigger_msg_type_ == "int32") {
      int_trigger_sub_ = create_subscription<std_msgs::msg::Int32>(
        trigger_topic_, 10, std::bind(&AliyunImageAnalyzer::on_int_trigger, this, std::placeholders::_1));
    } else {
      bool_trigger_sub_ = create_subscription<std_msgs::msg::Bool>(
        trigger_topic_, 10, std::bind(&AliyunImageAnalyzer::on_bool_trigger, this, std::placeholders::_1));
    }

    RCLCPP_INFO(
      get_logger(), "aliyun_image_analyzer started: image=%s type=%s trigger=%s(%s) result=%s model=%s",
      image_topic_.c_str(), image_msg_type_.c_str(), trigger_topic_.c_str(), trigger_msg_type_.c_str(),
      result_topic_.c_str(), model_.c_str());
  }

  ~AliyunImageAnalyzer() override
  {
    processing_ = false;
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
  }

private:
  void on_compressed_image(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    latest_image_.assign(msg->data.begin(), msg->data.end());
    latest_image_stamp_ = now();
  }

  void on_raw_image(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      cv::Mat image = raw_msg_to_mat(*msg);
      std::vector<unsigned char> encoded;
      if (!cv::imencode(".jpg", image, encoded)) {
        RCLCPP_ERROR(get_logger(), "Failed to encode raw Image to JPEG.");
        return;
      }
      std::lock_guard<std::mutex> lock(image_mutex_);
      latest_image_ = std::move(encoded);
      latest_image_stamp_ = now();
    } catch (const std::exception & exc) {
      RCLCPP_ERROR(get_logger(), "Failed to convert raw Image: %s", exc.what());
    }
  }

  cv::Mat raw_msg_to_mat(const sensor_msgs::msg::Image & msg) const
  {
    if (msg.height == 0 || msg.width == 0 || msg.data.empty()) {
      throw std::runtime_error("empty raw image");
    }

    if (msg.encoding == "bgr8") {
      return cv::Mat(msg.height, msg.width, CV_8UC3, const_cast<unsigned char *>(msg.data.data()), msg.step).clone();
    }
    if (msg.encoding == "rgb8") {
      cv::Mat rgb(msg.height, msg.width, CV_8UC3, const_cast<unsigned char *>(msg.data.data()), msg.step);
      cv::Mat bgr;
      cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
      return bgr;
    }
    if (msg.encoding == "mono8" || msg.encoding == "8UC1") {
      return cv::Mat(msg.height, msg.width, CV_8UC1, const_cast<unsigned char *>(msg.data.data()), msg.step).clone();
    }

    throw std::runtime_error("unsupported raw image encoding: " + msg.encoding);
  }

  void on_bool_trigger(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      request_analysis();
    }
  }

  void on_int_trigger(const std_msgs::msg::Int32::SharedPtr msg)
  {
    if (msg->data == trigger_value_) {
      request_analysis();
    }
  }

  void request_analysis()
  {
    const auto now_time = now();
    if (processing_.exchange(true)) {
      RCLCPP_WARN(get_logger(), "Previous image analysis is still running; trigger ignored.");
      return;
    }
    if (has_last_request_time_ && (now_time - last_request_time_).seconds() < min_interval_sec_) {
      processing_ = false;
      RCLCPP_WARN(get_logger(), "Image analysis trigger is too frequent; trigger ignored.");
      return;
    }

    std::vector<unsigned char> image;
    {
      std::lock_guard<std::mutex> lock(image_mutex_);
      image = latest_image_;
    }
    if (image.empty()) {
      processing_ = false;
      RCLCPP_ERROR(get_logger(), "No image has been received; cannot analyze.");
      return;
    }

    const std::string api_key = get_api_key();
    if (api_key.empty()) {
      processing_ = false;
      RCLCPP_ERROR(
        get_logger(), "DashScope API key is empty. Set parameter api_key or environment variable %s.",
        api_key_env_.c_str());
      return;
    }

    last_request_time_ = now_time;
    has_last_request_time_ = true;
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
    worker_thread_ = std::thread([this, api_key, image = std::move(image)]() {
      try {
        if (save_debug_image_) {
          save_debug_image(image);
        }
        const std::string result = call_dashscope(api_key, image);
        std_msgs::msg::String out;
        out.data = publish_prefix_ + result;
        result_pub_->publish(out);
        RCLCPP_INFO(get_logger(), "Image analysis complete: %s", result.c_str());
      } catch (const std::exception & exc) {
        RCLCPP_ERROR(get_logger(), "Image analysis failed: %s", exc.what());
      }
      processing_ = false;
    });
  }

  std::string get_api_key() const
  {
    if (!trim(api_key_).empty()) {
      return trim(api_key_);
    }
    const char * value = std::getenv(api_key_env_.c_str());
    return value == nullptr ? "" : trim(value);
  }

  std::string call_dashscope(const std::string & api_key, const std::vector<unsigned char> & image) const
  {
    const std::string image_url =
      "data:" + image_mime_ + ";base64," + base64_encode(image);
    const std::string payload =
      "{\"model\":\"" + json_escape(model_) + "\",\"messages\":[{\"role\":\"user\",\"content\":["
      "{\"type\":\"text\",\"text\":\"" + json_escape(prompt_) + "\"},"
      "{\"type\":\"image_url\",\"image_url\":{\"url\":\"" + json_escape(image_url) + "\"}}"
      "]}]}";

    CURL * curl = curl_easy_init();
    if (curl == nullptr) {
      throw std::runtime_error("curl_easy_init failed");
    }

    std::string response_body;
    struct curl_slist * headers = nullptr;
    headers = curl_slist_append(headers, ("Authorization: Bearer " + api_key).c_str());
    headers = curl_slist_append(headers, "Content-Type: application/json");

    curl_easy_setopt(curl, CURLOPT_URL, base_url_.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POST, 1L);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, payload.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, static_cast<long>(payload.size()));
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, static_cast<long>(request_timeout_sec_));
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, dashscope_write_callback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_body);

    const CURLcode code = curl_easy_perform(curl);
    long http_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);

    if (code != CURLE_OK) {
      throw std::runtime_error(std::string("curl request failed: ") + curl_easy_strerror(code));
    }
    if (http_code < 200 || http_code >= 300) {
      throw std::runtime_error("HTTP " + std::to_string(http_code) + ": " + response_body);
    }

    return extract_dashscope_content(response_body);
  }

  void save_debug_image(const std::vector<unsigned char> & image) const
  {
    std::filesystem::create_directories(debug_image_dir_);

    const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    const std::string path = debug_image_dir_ + "/image_" + std::to_string(millis) + ".jpg";
    std::ofstream out(path, std::ios::binary);
    out.write(reinterpret_cast<const char *>(image.data()), static_cast<std::streamsize>(image.size()));
    RCLCPP_INFO(get_logger(), "Saved debug image: %s", path.c_str());
  }

  std::string image_topic_;
  std::string image_msg_type_;
  std::string trigger_topic_;
  std::string trigger_msg_type_;
  std::string result_topic_;
  std::string api_key_;
  std::string api_key_env_;
  std::string base_url_;
  std::string model_;
  std::string prompt_;
  std::string image_mime_;
  std::string publish_prefix_;
  std::string debug_image_dir_;
  int trigger_value_{30};
  double request_timeout_sec_{20.0};
  double min_interval_sec_{3.0};
  bool save_debug_image_{false};

  std::mutex image_mutex_;
  std::vector<unsigned char> latest_image_;
  rclcpp::Time latest_image_stamp_;
  rclcpp::Time last_request_time_{0, 0, RCL_ROS_TIME};
  bool has_last_request_time_{false};
  std::atomic<bool> processing_{false};
  std::thread worker_thread_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_image_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bool_trigger_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr int_trigger_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AliyunImageAnalyzer>());
  rclcpp::shutdown();
  return 0;
}
