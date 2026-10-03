#pragma once
#include "pros/abstract_motor.hpp"

namespace ROBOT_CONSTANTS {
enum class TransmissionGear { FIRST, SECOND, THIRD, FOURTH };
inline constexpr std::int32_t C_JOYSTICK_DEADBAND{3};
inline constexpr std::int32_t C_MAX_DRIVE_MOTOR_VOLTAGE_MV{12000};
inline constexpr std::uint32_t C_TRANSMISSION_SHIFT_TIME_MS{50};
inline constexpr std::uint32_t C_TRANSMISSION_ZERO_CURRENT_MA{2500};
inline constexpr std::uint32_t C_TRANSMISSION_ZERO_TIMEOUT_MS{3000};
inline constexpr std::int32_t C_TRANSMISSION_MAX_VOLTAGE_MV{6000};
inline constexpr std::int32_t C_TRANSMISSION_HOMING_VOLTAGE{2000};
inline constexpr std::uint8_t C_TRANSMISSION_HOMING_ZERO_SPEED{5};
inline constexpr std::int32_t C_TRANSMISSION_MAX_SPEED{50};
inline constexpr std::array<std::pair<TransmissionGear, double>, 4>
    TRANSMISSION_ENCODER_POSITIONS{
        std::make_pair(TransmissionGear::FIRST, 0.0),
        std::make_pair(TransmissionGear::SECOND, 100.0),
        std::make_pair(TransmissionGear::THIRD, 200.0),
        std::make_pair(TransmissionGear::FOURTH, 300.0)};
inline constexpr std::initializer_list<int8_t> TRANSMISSION_MOTORS{9, 10};
inline constexpr std::initializer_list<int8_t> LEFT_DRIVE_MOTORS{1, 2, 3, 4};
inline constexpr std::initializer_list<int8_t> RIGHT_DRIVE_MOTORS{-5, -6, -7,
                                                                  -8};
inline constexpr pros::MotorGear DRIVE_GEARING{pros::MotorGear::green};
inline constexpr pros::MotorBrake DRIVE_BRAKE_MODE{pros::MotorBrake::brake};

inline constexpr pros::MotorGear SHIFT_GEARINNG{pros::MotorGear::green};
inline constexpr pros::MotorBrake SHIFT_BRAKE_MODE{pros::MotorBrake::hold};
}; // namespace ROBOT_CONSTANTS
