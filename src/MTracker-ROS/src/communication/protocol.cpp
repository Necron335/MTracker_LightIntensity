#include "mtracker/communication/protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <limits>

namespace mtracker
{
namespace
{
void append_u16(std::vector<std::uint8_t> & out, std::uint16_t value)
{
  out.push_back(static_cast<std::uint8_t>(value));
  out.push_back(static_cast<std::uint8_t>(value >> 8));
}

std::int16_t read_i16(const std::uint8_t * data)
{
  return static_cast<std::int16_t>(
    static_cast<std::uint16_t>(data[0]) | static_cast<std::uint16_t>(data[1]) << 8);
}

void append_float(std::vector<std::uint8_t> & out, float value)
{
  static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
    "Protocol requires IEEE-754 binary32");
  if (!std::isfinite(value)) throw std::invalid_argument("Nonfinite command coordinate");
  std::uint32_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  for (unsigned shift = 0; shift < 32; shift += 8) {
    out.push_back(static_cast<std::uint8_t>(bits >> shift));
  }
}

float read_float(const std::uint8_t * data)
{
  std::uint32_t bits = 0;
  for (unsigned i = 0; i < 4; ++i) {
    bits |= static_cast<std::uint32_t>(data[i]) << (8 * i);
  }
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

std::int32_t scaled(float value, float scale, std::int32_t low, std::int32_t high)
{
  if (!std::isfinite(value)) throw std::invalid_argument("Nonfinite wheel velocity");
  const double result = std::clamp(static_cast<double>(value) * scale,
    static_cast<double>(low), static_cast<double>(high));
  return static_cast<std::int32_t>(std::lround(result));
}
}  // namespace

FrameParser::FrameParser(std::uint16_t robot_address, std::chrono::milliseconds timeout)
: robot_address_(robot_address), timeout_(timeout) {}

void FrameParser::reset()
{
  state_ = State::kMarker;
  content_length_ = 0;
  index_ = 0;
  received_crc_ = 0;
}

void FrameParser::expire(Clock::time_point now)
{
  if (state_ != State::kMarker && now - last_byte_ >= timeout_) {
    ++counters_.incomplete_timeouts;
    reset();
  }
}

std::optional<Frame> FrameParser::feed_byte(std::uint8_t byte, Clock::time_point now)
{
  expire(now);
  if (state_ != State::kMarker || byte == Protocol::kHeader) last_byte_ = now;
  switch (state_) {
    case State::kMarker:
      if (byte == Protocol::kHeader) state_ = State::kLength;
      break;
    case State::kLength:
      if (byte < 4 || byte > 31) {
        ++counters_.format_errors;
        reset();
        if (byte == Protocol::kHeader) state_ = State::kLength;
      } else {
        content_length_ = static_cast<std::size_t>(byte) + 1;
        content_[0] = byte;
        index_ = 1;
        state_ = State::kContent;
      }
      break;
    case State::kContent:
      content_[index_++] = byte;
      if (index_ == content_length_) state_ = State::kCrcHigh;
      break;
    case State::kCrcHigh:
      received_crc_ = static_cast<std::uint16_t>(byte) << 8;
      state_ = State::kCrcLow;
      break;
    case State::kCrcLow: {
        received_crc_ |= byte;
        if (received_crc_ != Protocol::crc16(content_.data(), content_length_)) {
          ++counters_.crc_errors;
          reset();
          break;
        }
        const auto address = static_cast<std::uint16_t>(content_[3]) |
          static_cast<std::uint16_t>(content_[4]) << 8;
        // The directly connected firmware returns address zero in its replies.
        // Nonzero addresses must still match the configured destination mask.
        if (content_[1] != Protocol::kGroup || (address != 0 && address != robot_address_)) {
          ++counters_.format_errors;
          reset();
          break;
        }
        Frame frame{content_[1], content_[2], static_cast<std::uint16_t>(
          content_[3] | static_cast<std::uint16_t>(content_[4]) << 8), {}};
        frame.data.assign(content_.begin() + 5, content_.begin() + content_length_);
        ++counters_.valid_frames;
        reset();
        return frame;
      }
  }
  return std::nullopt;
}

std::vector<Frame> FrameParser::feed(
  const std::uint8_t * data, std::size_t size, Clock::time_point now)
{
  std::vector<Frame> frames;
  for (std::size_t i = 0; i < size; ++i) {
    if (auto frame = feed_byte(data[i], now)) frames.push_back(std::move(*frame));
  }
  return frames;
}

std::vector<Frame> FrameParser::feed(const std::vector<std::uint8_t> & data, Clock::time_point now)
{
  return feed(data.data(), data.size(), now);
}

std::uint16_t Protocol::crc16(const std::uint8_t * data, std::size_t size)
{
  std::uint16_t crc = 0xFFFF;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= static_cast<std::uint16_t>(data[i]) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) != 0 ?
        static_cast<std::uint16_t>((crc << 1) ^ 0x8005U) :
        static_cast<std::uint16_t>(crc << 1);
    }
  }
  return crc;
}

std::vector<std::uint8_t> Protocol::encode_frame(const Frame & frame)
{
  if (frame.data.size() > 27 || frame.robot == 0) return {};
  std::vector<std::uint8_t> out{kHeader, static_cast<std::uint8_t>(4 + frame.data.size()),
    frame.group, frame.command, static_cast<std::uint8_t>(frame.robot),
    static_cast<std::uint8_t>(frame.robot >> 8)};
  out.insert(out.end(), frame.data.begin(), frame.data.end());
  const auto crc = crc16(out.data() + 1, out.size() - 1);
  out.push_back(static_cast<std::uint8_t>(crc >> 8));
  out.push_back(static_cast<std::uint8_t>(crc));
  return out;
}

std::vector<std::uint8_t> Protocol::encode_wheel_command(
  std::uint16_t robot, const WheelCommand & command)
{
  Frame frame{kGroup, kWheelCommand, robot, {}};
  append_u16(frame.data, command.status);
  append_u16(frame.data, static_cast<std::uint16_t>(scaled(command.left_velocity, 256, -32768, 32767)));
  append_u16(frame.data, static_cast<std::uint16_t>(scaled(command.right_velocity, 256, -32768, 32767)));
  append_float(frame.data, command.x);
  append_float(frame.data, command.y);
  append_float(frame.data, command.theta);
  return encode_frame(frame);
}

std::int32_t Protocol::decode_int24(const std::uint8_t * data)
{
    const std::int32_t bits = data[0] | (static_cast<std::int32_t>(data[1]) << 8) |
        (static_cast<std::int32_t>(data[2]) << 16);
    return (bits & 0x800000) ? bits - 0x1000000 : bits;
}

std::vector<std::uint8_t> Protocol::encode_compact_command(
    std::uint16_t robot, const WheelCommand & command)
{
    Frame frame{kGroup, 6, robot, {}};
    append_u16(frame.data, command.status);
    append_u16(frame.data, static_cast<std::uint16_t>(scaled(command.left_velocity, 256, -32768, 32767)));
    append_u16(frame.data, static_cast<std::uint16_t>(scaled(command.right_velocity, 256, -32768, 32767)));
    for (const float coordinate : {command.x, command.y}) {
        const auto bits = static_cast<std::uint32_t>(scaled(coordinate, 32768, -8388608, 8388607));
        for (unsigned shift = 0; shift < 24; shift += 8) {
            frame.data.push_back(static_cast<std::uint8_t>(bits >> shift));
        }
    }
    append_u16(frame.data, static_cast<std::uint16_t>(scaled(command.theta, 1024, -32768, 32767)));
    append_u16(frame.data, 0);  // RESERVED is mandatory: DATA=16 bytes, LEN=20.
    return encode_frame(frame);
}

std::optional<Telemetry> Protocol::decode_wheel_telemetry(const Frame & frame)
{
  if (frame.group == kGroup && frame.command == 6 &&
    (frame.data.size() == 23
#ifdef LIGHT_INTENSITY_BH1750
    || frame.data.size() == 26
#endif
    ))
  {
    const auto & d = frame.data;
    Telemetry t;
    t.left_wheel_velocity = read_i16(&d[0]) / 256.0F;
    t.right_wheel_velocity = read_i16(&d[2]) / 256.0F;
    t.x = decode_int24(&d[4]) / 32768.0F;
    t.y = decode_int24(&d[7]) / 32768.0F;
    t.theta = read_i16(&d[10]) / 1024.0F;
    t.left_error = read_i16(&d[12]) / 1024.0F;
    t.right_error = read_i16(&d[14]) / 1024.0F;
    t.left_control = (d[16] < 128 ? int(d[16]) : int(d[16]) - 256) / 127.0F;
    t.right_control = (d[17] < 128 ? int(d[17]) : int(d[17]) - 256) / 127.0F;
    const int voltage = (int(d[18]) << 4) | (d[19] >> 4);
    int current = ((int(d[19]) & 15) << 8) | d[20];
    if (current & 0x800) current -= 0x1000;
    const int power = (int(d[21]) << 4) | (d[22] >> 4);
    t.voltage = voltage * 0.01F;
    t.current = current * 0.002F;
    t.power = power * 0.025F;
    t.power_flags = d[22] & 15;
#ifdef LIGHT_INTENSITY_BH1750
    if (d.size() == 26) {
      // Firmware appends unsigned little-endian centilux; all ones means unavailable.
      const std::uint32_t light = static_cast<std::uint32_t>(d[23]) |
        (static_cast<std::uint32_t>(d[24]) << 8) |
        (static_cast<std::uint32_t>(d[25]) << 16);
      t.light_valid = light != 0xFFFFFFU;
      if (t.light_valid) t.illuminance_lux = light / 100.0F;
    }
#endif
    return t;
  }
  // Legacy command 3: status, wheels, float32 posture, supply mV, UR Q12.
  if (frame.group != kGroup || frame.command != kWheelCommand || frame.data.size() != 22) {
    return std::nullopt;
  }
  Telemetry telemetry{
    read_i16(&frame.data[2]) / 256.0F, read_i16(&frame.data[4]) / 256.0F,
    read_float(&frame.data[6]), read_float(&frame.data[10]), read_float(&frame.data[14]),
    0.0F, read_i16(&frame.data[20]) / 4096.0F};
  telemetry.voltage = (static_cast<std::uint16_t>(frame.data[18]) |
    static_cast<std::uint16_t>(frame.data[19]) << 8) / 1000.0F;
  if (!std::isfinite(telemetry.x) || !std::isfinite(telemetry.y) ||
    !std::isfinite(telemetry.theta))
  {
    return std::nullopt;
  }
  return telemetry;
}

}  // namespace mtracker
