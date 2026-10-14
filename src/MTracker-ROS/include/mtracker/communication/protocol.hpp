#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>
#ifdef LIGHT_INTENSITY_BH1750
#include <limits>
#endif

namespace mtracker
{

struct Frame
{
  std::uint8_t group{};
  std::uint8_t command{};
  std::uint16_t robot{};
  std::vector<std::uint8_t> data;
};

struct ParserCounters
{
  std::uint64_t valid_frames{};
  std::uint64_t crc_errors{};
  std::uint64_t format_errors{};
  std::uint64_t incomplete_timeouts{};
};

class FrameParser
{
public:
  using Clock = std::chrono::steady_clock;
  explicit FrameParser(std::uint16_t robot_address, std::chrono::milliseconds timeout =
    std::chrono::milliseconds(10));
  std::vector<Frame> feed(const std::uint8_t * data, std::size_t size, Clock::time_point now);
  std::vector<Frame> feed(const std::vector<std::uint8_t> & data, Clock::time_point now);
  void expire(Clock::time_point now);
  const ParserCounters & counters() const {return counters_;}

private:
  enum class State {kMarker, kLength, kContent, kCrcHigh, kCrcLow};
  void reset();
  std::optional<Frame> feed_byte(std::uint8_t byte, Clock::time_point now);
  State state_{State::kMarker};
  std::array<std::uint8_t, 32> content_{};
  std::size_t content_length_{};
  std::size_t index_{};
  std::uint16_t received_crc_{};
  std::uint16_t robot_address_;
  std::chrono::milliseconds timeout_;
  Clock::time_point last_byte_{};
  ParserCounters counters_{};
};

struct WheelCommand
{
  std::uint16_t status{};
  float left_velocity{};
  float right_velocity{};
  float x{};
  float y{};
  float theta{};
};

struct Telemetry
{
  float left_wheel_velocity{};
  float right_wheel_velocity{};
  float x{};
  float y{};
  float theta{};
  float left_control{};
  float right_control{};
  float left_error{};
  float right_error{};
  float voltage{};
  float current{};
  float power{};
  std::uint8_t power_flags{};
  bool power_valid() const {return (power_flags & 1U) != 0;}
#ifdef LIGHT_INTENSITY_BH1750
  float illuminance_lux{std::numeric_limits<float>::quiet_NaN()};
  bool light_valid{};
#endif
};

class Protocol
{
public:
  static constexpr std::uint8_t kHeader = 0xAA;
  static constexpr std::uint8_t kGroup = 1;
  static constexpr std::uint8_t kWheelCommand = 3;
  static std::uint16_t crc16(const std::uint8_t * data, std::size_t size);
  static std::vector<std::uint8_t> encode_frame(const Frame & frame);
  static std::vector<std::uint8_t> encode_wheel_command(
    std::uint16_t robot, const WheelCommand & command);
  static std::vector<std::uint8_t> encode_compact_command(
    std::uint16_t robot, const WheelCommand & command);
  static std::int32_t decode_int24(const std::uint8_t * data);
  static std::optional<Telemetry> decode_wheel_telemetry(const Frame & frame);
};

}  // namespace mtracker
