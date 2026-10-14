#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "mtracker/communication/protocol.hpp"
#include "mtracker/communication/serial_port.hpp"

namespace mtracker
{
using namespace std::chrono_literals;

class DriverNode : public rclcpp::Node
{
public:
  DriverNode()
  : Node("mtracker"),
    device_(declare_parameter<std::string>("device", "/dev/ttyACM0")),
    baud_rate_(declare_parameter<int>("baud_rate", 921600)),
    robot_address_(declare_parameter<int>("robot_address", 0x0100)),
    enable_drive_(declare_parameter<bool>("enable_drive", true)),
    command_type_(declare_parameter<int>("command_type", 6)),
    response_timeout_(declare_parameter<int>("response_timeout_ms", 50)),
    wheel_base_(declare_parameter<double>("wheel_base", 0.15)),
    wheel_radius_(declare_parameter<double>("wheel_radius", 0.025)),
    command_timeout_(declare_parameter<double>("command_timeout", 0.5)),
    odometry_topic_(declare_parameter<std::string>("odometry_topic", "/wheel/odom")),
    odom_frame_(declare_parameter<std::string>(
      "odom_frame_id", declare_parameter<std::string>("odom_frame", "odom"))),
    base_frame_(declare_parameter<std::string>(
      "frame_id", declare_parameter<std::string>("base_frame", "base_link"))),
    parser_(checked_robot_address()), transform_broadcaster_(*this),
    pose_sigma_x_(declare_parameter<double>("pose_sigma_x", 0.01)),
    pose_sigma_y_(declare_parameter<double>("pose_sigma_y", 0.01)),
    pose_sigma_theta_(declare_parameter<double>("pose_sigma_theta", 0.02)),
    twist_sigma_linear_(declare_parameter<double>("twist_sigma_linear", 0.02)),
    twist_sigma_angular_(declare_parameter<double>("twist_sigma_angular", 0.03))
  {
    if (command_type_ != 3 && command_type_ != 6) {
      throw std::invalid_argument("command_type must be 3 or 6");
    }
    voltage_pub_ = create_publisher<std_msgs::msg::Float32>("voltage", 10);
    current_pub_ = create_publisher<std_msgs::msg::Float32>("current", 10);
    power_pub_ = create_publisher<std_msgs::msg::Float32>("power", 10);
    power_valid_pub_ = create_publisher<std_msgs::msg::Bool>("power_valid", 10);
    power_flags_pub_ = create_publisher<std_msgs::msg::UInt8>("power_flags", 10);
#ifdef LIGHT_INTENSITY_BH1750
    light_intensity_pub_ = create_publisher<std_msgs::msg::Float32>("light_intensity", 10);
    light_valid_pub_ = create_publisher<std_msgs::msg::Bool>("light_intensity_valid", 10);
#endif
    const auto legacy_device = declare_parameter<std::string>("serial_port", "");
    if (!legacy_device.empty()) device_ = legacy_device;
    const double rate = declare_parameter<double>("command_rate_hz", 100.0);
    if (rate <= 0.0) throw std::invalid_argument("command_rate_hz must be positive");
    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>(odometry_topic_, 10);
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);
    command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {handle_command(*msg);});
    odometry_reset_subscription_ = create_subscription<geometry_msgs::msg::Pose2D>(
      "set_odometry", 1, [this](geometry_msgs::msg::Pose2D::ConstSharedPtr msg) {
        pending_odometry_ = *msg;
      });
    last_command_time_ = now();
    last_reconnect_attempt_ = std::chrono::steady_clock::now() - 2s;
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate)),
      [this]() {update();});
    diagnostics_timer_ = create_wall_timer(1s, [this]() {publish_diagnostics();});
  }

  ~DriverNode() override
  {
    if (serial_.is_open()) {
      try {
        const auto frame = encode_command(WheelCommand{});
        serial_.write_all(frame.data(), frame.size());
      } catch (const std::exception &) {}
    }
  }

private:
  std::vector<std::uint8_t> encode_command(const WheelCommand & command) const
  {
    return command_type_ == 6 ? Protocol::encode_compact_command(checked_robot_address(), command) :
           Protocol::encode_wheel_command(checked_robot_address(), command);
  }

  std::uint16_t checked_robot_address() const
  {
    if (robot_address_ < 1 || robot_address_ > 65535) throw std::invalid_argument("robot_address must be a nonzero 16-bit mask");
    return static_cast<std::uint16_t>(robot_address_);
  }

  void handle_command(const geometry_msgs::msg::Twist & command)
  {
    const double half_base = wheel_base_ / 2.0;
    left_command_ = static_cast<float>((command.linear.x - half_base * command.angular.z) / wheel_radius_);
    right_command_ = static_cast<float>((command.linear.x + half_base * command.angular.z) / wheel_radius_);
    last_command_time_ = now();
  }

  void disconnect(const std::string & reason)
  {
    if (serial_.is_open()) {
      ++disconnects_;
      RCLCPP_WARN(get_logger(), "Serial connection lost: %s", reason.c_str());
    }
    serial_.close_port();
    awaiting_response_ = false;
    communication_confirmed_ = false;
  }

  void reconnect(std::chrono::steady_clock::time_point steady_now)
  {
    if (steady_now - last_reconnect_attempt_ < 1s) return;
    last_reconnect_attempt_ = steady_now;
    try {
      serial_.open_port(device_, baud_rate_);
      ++reconnections_;
      RCLCPP_INFO(get_logger(), "Connected to %s", device_.c_str());
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.what());
    }
  }

  void update()
  {
    const auto steady_now = std::chrono::steady_clock::now();
    if (!serial_.is_open()) {
      reconnect(steady_now);
      return;
    }
    try {
      std::vector<std::uint8_t> bytes;
      serial_.read_available(bytes);
      for (const auto & frame : parser_.feed(bytes, steady_now)) {
        if (frame.command != command_type_) continue;
        if (const auto telemetry = Protocol::decode_wheel_telemetry(frame)) {
          awaiting_response_ = false;
          communication_confirmed_ = true;
          publish(*telemetry);
        }
      }
      parser_.expire(steady_now);
      if (awaiting_response_) {
        if (steady_now - request_time_ >= response_timeout_) {
          ++response_timeouts_;
          awaiting_response_ = false;
          communication_confirmed_ = false;
        } else {
          return;
        }
      }
      WheelCommand command;
      if (pending_odometry_) {
        command.status = 0x0004;
        command.x = static_cast<float>(pending_odometry_->x);
        command.y = static_cast<float>(pending_odometry_->y);
        command.theta = static_cast<float>(pending_odometry_->theta);
        pending_odometry_.reset();
      } else if (enable_drive_ && communication_confirmed_ &&
        (now() - last_command_time_).seconds() <= command_timeout_)
      {
        command.status = 0x0003;
        command.left_velocity = left_command_;
        command.right_velocity = right_command_;
      }
      const auto encoded = encode_command(command);
      serial_.write_all(encoded.data(), encoded.size());
      awaiting_response_ = true;
      request_time_ = steady_now;
    } catch (const std::exception & error) {
      disconnect(error.what());
    }
  }

  void publish(const Telemetry & telemetry)
  {
    std_msgs::msg::Float32 value;
    value.data = telemetry.voltage;
    voltage_pub_->publish(value);
    value.data = telemetry.power_valid() ? telemetry.current : std::numeric_limits<float>::quiet_NaN();
    current_pub_->publish(value);
    value.data = telemetry.power_valid() ? telemetry.power : std::numeric_limits<float>::quiet_NaN();
    power_pub_->publish(value);
    std_msgs::msg::Bool valid;
    valid.data = telemetry.power_valid();
    power_valid_pub_->publish(valid);
#ifdef LIGHT_INTENSITY_BH1750
    value.data = telemetry.illuminance_lux;
    light_intensity_pub_->publish(value);
    valid.data = telemetry.light_valid;
    light_valid_pub_->publish(valid);
#endif
    std_msgs::msg::UInt8 flags;
    flags.data = telemetry.power_flags;
    power_flags_pub_->publish(flags);
    const auto stamp = now();
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = odom_frame_;
    odometry.child_frame_id = base_frame_;
    odometry.pose.pose.position.x = telemetry.x;
    odometry.pose.pose.position.y = telemetry.y;
    tf2::Quaternion rotation;
    rotation.setRPY(0.0, 0.0, telemetry.theta);
    odometry.pose.pose.orientation = tf2::toMsg(rotation);
    odometry.twist.twist.linear.x = wheel_radius_ *
      (telemetry.left_wheel_velocity + telemetry.right_wheel_velocity) / 2.0;
    odometry.twist.twist.angular.z = wheel_radius_ *
      (telemetry.right_wheel_velocity - telemetry.left_wheel_velocity) / wheel_base_;
    odometry.pose.covariance.fill(0.0);
    odometry.pose.covariance[0] = pose_sigma_x_ * pose_sigma_x_;
    odometry.pose.covariance[7] = pose_sigma_y_ * pose_sigma_y_;
    odometry.pose.covariance[14] = odometry.pose.covariance[21] =
      odometry.pose.covariance[28] = 1.0e6;
    odometry.pose.covariance[35] = pose_sigma_theta_ * pose_sigma_theta_;
    odometry.twist.covariance.fill(0.0);
    odometry.twist.covariance[0] = twist_sigma_linear_ * twist_sigma_linear_;
    odometry.twist.covariance[7] = odometry.twist.covariance[14] =
      odometry.twist.covariance[21] = odometry.twist.covariance[28] = 1.0e6;
    odometry.twist.covariance[35] = twist_sigma_angular_ * twist_sigma_angular_;
    odometry_publisher_->publish(odometry);
    geometry_msgs::msg::TransformStamped transform;
    transform.header = odometry.header;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = telemetry.x;
    transform.transform.translation.y = telemetry.y;
    transform.transform.rotation = odometry.pose.pose.orientation;
    transform_broadcaster_.sendTransform(transform);
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "mtracker/serial";
    status.hardware_id = device_;
    status.level = communication_confirmed_ ? status.OK : status.WARN;
    status.message = communication_confirmed_ ? "communicating" : "no confirmed response";
    const auto & counters = parser_.counters();
    const auto add = [&status](const std::string & key, std::uint64_t value) {
        diagnostic_msgs::msg::KeyValue item;
        item.key = key;
        item.value = std::to_string(value);
        status.values.push_back(std::move(item));
      };
    add("valid_frames", counters.valid_frames);
    add("crc_errors", counters.crc_errors);
    add("format_errors", counters.format_errors);
    add("incomplete_frame_timeouts", counters.incomplete_timeouts);
    add("response_timeouts", response_timeouts_);
    add("disconnects", disconnects_);
    add("reconnections", reconnections_);
    array.status.push_back(std::move(status));
    diagnostics_publisher_->publish(array);
  }

  std::string device_;
  const int baud_rate_, robot_address_;
  const bool enable_drive_;
  const int command_type_;
  const std::chrono::milliseconds response_timeout_;
  const double wheel_base_, wheel_radius_, command_timeout_;
  const std::string odometry_topic_, odom_frame_, base_frame_;
  SerialPort serial_;
  FrameParser parser_;
  tf2_ros::TransformBroadcaster transform_broadcaster_;
  const double pose_sigma_x_, pose_sigma_y_, pose_sigma_theta_;
  const double twist_sigma_linear_, twist_sigma_angular_;
  float left_command_{}, right_command_{};
  rclcpp::Time last_command_time_;
  std::optional<geometry_msgs::msg::Pose2D> pending_odometry_;
  bool awaiting_response_{}, communication_confirmed_{};
  std::chrono::steady_clock::time_point request_time_{}, last_reconnect_attempt_{};
  std::uint64_t response_timeouts_{}, disconnects_{}, reconnections_{};
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_pub_, current_pub_, power_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr power_valid_pub_;
#ifdef LIGHT_INTENSITY_BH1750
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr light_intensity_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr light_valid_pub_;
#endif
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr power_flags_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr odometry_reset_subscription_;
  rclcpp::TimerBase::SharedPtr timer_, diagnostics_timer_;
};
}  // namespace mtracker

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<mtracker::DriverNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("mtracker"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
