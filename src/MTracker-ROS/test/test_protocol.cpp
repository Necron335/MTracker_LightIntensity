#include <cstdint>
#include <cmath>
#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "mtracker/communication/protocol.hpp"

namespace
{
// Payload from the packed layout of the original Serial.hpp (firmware 2.53).
mtracker::Frame telemetry_frame()
{
    return {1, 3, 0x0100, {
        3, 0, 0, 1, 0, 254,  // status=3, left=1 rad/s, right=-2 rad/s
        0, 0, 128, 63,       // x=1.0 (IEEE-754 little endian)
        0, 0, 0, 192,        // y=-2.0
        0, 0, 0, 63,         // theta=0.5
        123, 0, 133, 255}};  // control=123, -123
}
}  // namespace

TEST(Protocol, ExtendedLightTelemetry)
{
    mtracker::Frame frame{1, 6, 0x0100, std::vector<std::uint8_t>(26, 0)};
    frame.data[23] = 0x87;
    frame.data[24] = 0xD6;
    frame.data[25] = 0x12;  // 1234567 centilux; exercises all three bytes.
    const auto bytes = mtracker::Protocol::encode_frame(frame);
    ASSERT_EQ(bytes.size(), 34U);
    EXPECT_EQ(bytes[1], 30);
    mtracker::FrameParser parser(0x0100);
    const auto now = mtracker::FrameParser::Clock::now();
    EXPECT_TRUE(parser.feed(bytes.data(), 12, now).empty());
    const auto frames = parser.feed(bytes.data() + 12, bytes.size() - 12, now);
    ASSERT_EQ(frames.size(), 1U);
#ifdef LIGHT_INTENSITY_BH1750
    auto telemetry = mtracker::Protocol::decode_wheel_telemetry(frames[0]);
    ASSERT_TRUE(telemetry);
    EXPECT_TRUE(telemetry->light_valid);
    EXPECT_FLOAT_EQ(telemetry->illuminance_lux, 12345.67F);
    frame.data[23] = frame.data[24] = frame.data[25] = 0;
    telemetry = mtracker::Protocol::decode_wheel_telemetry(frame);
    ASSERT_TRUE(telemetry);
    EXPECT_TRUE(telemetry->light_valid);
    EXPECT_FLOAT_EQ(telemetry->illuminance_lux, 0);
    frame.data[23] = frame.data[24] = frame.data[25] = 0xFF;
    telemetry = mtracker::Protocol::decode_wheel_telemetry(frame);
    ASSERT_TRUE(telemetry);
    EXPECT_FALSE(telemetry->light_valid);
    EXPECT_TRUE(std::isnan(telemetry->illuminance_lux));
    frame.data.resize(23);
    telemetry = mtracker::Protocol::decode_wheel_telemetry(frame);
    ASSERT_TRUE(telemetry);
    EXPECT_FALSE(telemetry->light_valid);
    EXPECT_TRUE(std::isnan(telemetry->illuminance_lux));
    telemetry = mtracker::Protocol::decode_wheel_telemetry(telemetry_frame());
    ASSERT_TRUE(telemetry);
    EXPECT_FALSE(telemetry->light_valid);
    EXPECT_TRUE(std::isnan(telemetry->illuminance_lux));
#else
    EXPECT_FALSE(mtracker::Protocol::decode_wheel_telemetry(frames[0]));
#endif
    for (const auto size : {24U, 25U, 27U}) {
        frame.data.resize(size);
        EXPECT_FALSE(mtracker::Protocol::decode_wheel_telemetry(frame));
    }
}

TEST(Protocol, EncodesLegacyStopHeaderAndCrc)
{
    const auto bytes = mtracker::Protocol::encode_wheel_command(0x0100, {});
    ASSERT_EQ(bytes.size(), 26U);
    const std::vector<std::uint8_t> header{0xAA, 0x16, 1, 3, 0, 1};
    EXPECT_EQ(std::vector<std::uint8_t>(bytes.begin(), bytes.begin() + 6), header);
    for (std::size_t i = 6; i < 24; ++i) EXPECT_EQ(bytes[i], 0);
    const auto crc = mtracker::Protocol::crc16(bytes.data() + 1, 23);
    EXPECT_EQ(bytes[24], crc >> 8);
    EXPECT_EQ(bytes[25], crc & 0xFF);
    // Independent CRC-16/CMS check value for ASCII "123456789".
    const std::uint8_t check[]{'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    EXPECT_EQ(mtracker::Protocol::crc16(check, sizeof(check)), 0xAEE7);
}

TEST(Protocol, EncodesFloatPostureAndFixedPointWheelVelocities)
{
    const mtracker::WheelCommand command{4, 1.0F, -2.0F, 1.0F, -2.0F, 0.5F};
    const auto bytes = mtracker::Protocol::encode_wheel_command(0x0100, command);
    ASSERT_EQ(bytes.size(), 26U);
    const std::vector<std::uint8_t> payload{
        4, 0, 0, 1, 0, 254, 0, 0, 128, 63, 0, 0, 0, 192, 0, 0, 0, 63};
    EXPECT_EQ(std::vector<std::uint8_t>(bytes.begin() + 6, bytes.end() - 2), payload);
}

TEST(Protocol, AcceptsSplitTelemetryWithTwoByteAddress)
{
    mtracker::FrameParser parser(0x0100);
    const auto now = mtracker::FrameParser::Clock::now();
    const auto bytes = mtracker::Protocol::encode_frame(telemetry_frame());
    EXPECT_TRUE(parser.feed(std::vector<std::uint8_t>{1, 2, 3}, now).empty());
    EXPECT_TRUE(parser.feed(bytes.data(), 7, now).empty());
    const auto frames = parser.feed(bytes.data() + 7, bytes.size() - 7, now);
    ASSERT_EQ(frames.size(), 1U);
    EXPECT_EQ(frames[0].robot, 0x0100);
    const auto telemetry = mtracker::Protocol::decode_wheel_telemetry(frames[0]);
    ASSERT_TRUE(telemetry);
    EXPECT_FLOAT_EQ(telemetry->left_wheel_velocity, 1);
    EXPECT_FLOAT_EQ(telemetry->right_wheel_velocity, -2);
    EXPECT_FLOAT_EQ(telemetry->x, 1);
    EXPECT_FLOAT_EQ(telemetry->y, -2);
    EXPECT_FLOAT_EQ(telemetry->theta, 0.5);
    EXPECT_FLOAT_EQ(telemetry->voltage, 0.123F);
    EXPECT_FLOAT_EQ(telemetry->right_control, -123.0F / 4096);
    EXPECT_EQ(parser.counters().valid_frames, 1U);
}

TEST(Protocol, RejectsWrongAddressCrcAndIncompleteFrames)
{
    mtracker::FrameParser parser(0x0100);
    const auto now = mtracker::FrameParser::Clock::now();
    auto frame = telemetry_frame();
    frame.robot = 1;
    EXPECT_TRUE(parser.feed(mtracker::Protocol::encode_frame(frame), now).empty());
    EXPECT_EQ(parser.counters().format_errors, 1U);
    auto bytes = mtracker::Protocol::encode_frame(telemetry_frame());
    bytes.back() ^= 1;
    EXPECT_TRUE(parser.feed(bytes, now).empty());
    EXPECT_EQ(parser.counters().crc_errors, 1U);
    EXPECT_TRUE(parser.feed(bytes.data(), 3, now).empty());
    parser.expire(now + std::chrono::milliseconds(11));
    EXPECT_EQ(parser.counters().incomplete_timeouts, 1U);
    EXPECT_EQ(parser.feed(mtracker::Protocol::encode_frame(telemetry_frame()),
        now + std::chrono::milliseconds(12)).size(), 1U);
}

TEST(Protocol, RejectsAcknowledgmentsOtherCommandsAndNonfinitePostures)
{
    auto frame = telemetry_frame();
    frame.command = 6;
    EXPECT_FALSE(mtracker::Protocol::decode_wheel_telemetry(frame));
    frame.command = 3;
    frame.data.clear();
    EXPECT_FALSE(mtracker::Protocol::decode_wheel_telemetry(frame));
    frame = telemetry_frame();
    frame.data[8] = 0xC0;
    frame.data[9] = 0x7F;  // x=NaN
    EXPECT_FALSE(mtracker::Protocol::decode_wheel_telemetry(frame));
    mtracker::WheelCommand command;
    command.left_velocity = std::numeric_limits<float>::quiet_NaN();
    EXPECT_THROW(mtracker::Protocol::encode_wheel_command(0x0100, command), std::invalid_argument);
}

TEST(Protocol, DecodesActualRobotReplyWithZeroResponseAddress)
{
    // Captured from /dev/ttyUSB0 after a motors-off command on 2026-10-09.
    const std::vector<std::uint8_t> bytes{
        0xAA, 0x1A, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0xCB, 0xA9, 0x3E, 0x3B, 0x34, 0xA2, 0x56, 0xB8,
        0x56, 0x82, 0x1F, 0xBD, 0x4C, 0x4A, 0x00, 0x00, 0x4D, 0xDE};
    mtracker::FrameParser parser(0x0100);
    const auto frames = parser.feed(bytes, mtracker::FrameParser::Clock::now());
    ASSERT_EQ(frames.size(), 1U);
    const auto telemetry = mtracker::Protocol::decode_wheel_telemetry(frames[0]);
    ASSERT_TRUE(telemetry);
    EXPECT_FLOAT_EQ(telemetry->left_wheel_velocity, 0);
    EXPECT_FLOAT_EQ(telemetry->right_wheel_velocity, 0);
    EXPECT_NEAR(telemetry->x, 0.00290929, 1e-7);
    EXPECT_NEAR(telemetry->y, -0.0000511726, 1e-9);
    EXPECT_NEAR(telemetry->theta, -0.0389427, 1e-7);
    EXPECT_EQ(parser.counters().format_errors, 0U);
    EXPECT_EQ(parser.counters().crc_errors, 0U);
}

TEST(Protocol, CompactRequestContainsReservedAndSignedCoordinates)
{
    const auto bytes = mtracker::Protocol::encode_compact_command(
        0x0100, {4, 1, -2, -1, 1, -0.5F});
    ASSERT_EQ(bytes.size(), 24U);
    EXPECT_EQ(bytes[1], 20);
    EXPECT_EQ(bytes[3], 6);
    const std::vector<std::uint8_t> payload{
        4, 0, 0, 1, 0, 254, 0, 128, 255, 0, 128, 0, 0, 254, 0, 0};
    EXPECT_EQ(std::vector<std::uint8_t>(bytes.begin() + 6, bytes.end() - 2), payload);
    const std::uint8_t minimum[]{0, 0, 128}, maximum[]{255, 255, 127};
    EXPECT_EQ(mtracker::Protocol::decode_int24(minimum), -8388608);
    EXPECT_EQ(mtracker::Protocol::decode_int24(maximum), 8388607);
}

TEST(Protocol, DecodesActualCompactPowerReply)
{
    const std::vector<std::uint8_t> bytes{
        0xAA, 0x1B, 1, 6, 0, 1, 0, 0, 0, 0, 0x54, 0, 0,
        0xFF, 0xFF, 0xFF, 0xF2, 0xFF, 0xB9, 0x57, 0xFF, 0x7F,
        0, 0, 0x67, 0, 0, 0, 8, 0x1E, 0x73};
    mtracker::FrameParser parser(0x0100);
    std::vector<mtracker::Frame> frames;
    const auto now = mtracker::FrameParser::Clock::now();
    for (const auto byte : bytes) {
        const auto result = parser.feed(&byte, 1, now);
        frames.insert(frames.end(), result.begin(), result.end());
    }
    ASSERT_EQ(frames.size(), 1U);
    const auto telemetry = mtracker::Protocol::decode_wheel_telemetry(frames[0]);
    ASSERT_TRUE(telemetry);
    EXPECT_NEAR(telemetry->voltage, 16.48, 1e-5);
    EXPECT_EQ(telemetry->power_flags, 8);
    EXPECT_FALSE(telemetry->power_valid());
    EXPECT_FLOAT_EQ(telemetry->current, 0);
    EXPECT_FLOAT_EQ(telemetry->power, 0);
    EXPECT_FLOAT_EQ(telemetry->y, -1.0F / 32768);
    EXPECT_FLOAT_EQ(telemetry->theta, -14.0F / 1024);
}

TEST(Protocol, PackedPowerSignsFlagsAndLimits)
{
    mtracker::Frame frame{1, 6, 0x0100, std::vector<std::uint8_t>(23, 0)};
    // V=0xABC, I=0xFFF (-1), P=0x123, flags=0xF.
    frame.data[18] = 0xAB;
    frame.data[19] = 0xCF;
    frame.data[20] = 0xFF;
    frame.data[21] = 0x12;
    frame.data[22] = 0x3F;
    auto telemetry = mtracker::Protocol::decode_wheel_telemetry(frame);
    ASSERT_TRUE(telemetry);
    EXPECT_NEAR(telemetry->voltage, 27.48, 1e-5);
    EXPECT_NEAR(telemetry->current, -0.002, 1e-7);
    EXPECT_NEAR(telemetry->power, 7.275, 1e-5);
    EXPECT_TRUE(telemetry->power_valid());
    frame.data[19] = 0xC8;
    frame.data[20] = 0;
    EXPECT_NEAR(mtracker::Protocol::decode_wheel_telemetry(frame)->current, -4.096, 1e-6);
    frame.data[19] = 0xC7;
    frame.data[20] = 0xFF;
    EXPECT_NEAR(mtracker::Protocol::decode_wheel_telemetry(frame)->current, 4.094, 1e-6);
    frame.data.resize(21);
    EXPECT_FALSE(mtracker::Protocol::decode_wheel_telemetry(frame));
}

TEST(Protocol, CompactFramesCanContainMarkerAndArriveTogether)
{
    mtracker::Frame frame{1, 6, 0x0100, std::vector<std::uint8_t>(23, 0xAA)};
    auto bytes = mtracker::Protocol::encode_frame(frame);
    const auto second = bytes;
    bytes.insert(bytes.end(), second.begin(), second.end());
    mtracker::FrameParser parser(0x0100);
    EXPECT_EQ(parser.feed(bytes, mtracker::FrameParser::Clock::now()).size(), 2U);
}
