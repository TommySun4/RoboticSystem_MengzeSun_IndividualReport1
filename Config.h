#pragma once
#include <Arduino.h>
namespace Config {
constexpr int SDA_PIN = SDA, SCL_PIN = SCL, BUTTON_PIN = 3;
constexpr uint8_t ADDRESS = 0x08;
constexpr uint32_t I2C_HZ = 400000, SERIAL_BAUD = 115200;
constexpr uint16_t I2C_TIMEOUT_MS = 20;
constexpr uint32_t SAMPLE_MS = 10;
constexpr float COUNTS_PER_REV = 358.3f;
constexpr int PWM_LIMIT = 200; // Course interface limit; operating cap starts at 60.
// Raw channel signs; determine physical forward direction separately for each wheel.
constexpr int LEFT_MOTOR_SIGN = 1, RIGHT_MOTOR_SIGN = 1;
constexpr int LEFT_ENCODER_SIGN = 1, RIGHT_ENCODER_SIGN = 1;
// Matches teacher initialise(): name is optional in the example profile.
// Set true for course measurements that must use a configured team.
constexpr bool REQUIRE_TEAM = false;
constexpr uint32_t BOOT_SETTLE_MS = 50, BOOT_RETRY_MS = 50, BOOT_TIMEOUT_MS = 1000;
constexpr uint32_t PLOT_INTERVAL_MS = 50;
constexpr char TEAM_NAME[] = ""; // Your course team name, 1..31 printable ASCII chars.
}
