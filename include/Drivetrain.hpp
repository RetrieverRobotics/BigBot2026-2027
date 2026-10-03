#pragma once

#include "Config.hpp"
#include "Pros/motor_group.hpp"
#include <queue>
#include <utility>

enum class ControlState { DRIVER_CONTROL, AUTON_CONTROL, SHIFTING, NONE };

enum class GearState { FIRST, SECOND, THIRD, FOURTH, SHIFTING, NONE };

enum class ShiftRequestState { REQUESTED, SHIFTING, DONE, NONE };

enum class ShiftDirection { UP, DOWN, NONE };

struct Gear {
  int gear = 0;
  ROBOT_CONSTANTS::TransmissionGear currentGear =
      ROBOT_CONSTANTS::TransmissionGear::FIRST;
  double gearEncoderPos = 0.0;

  void next(void) {
    if (gear != 3) {
      gear++;
      currentGear = ROBOT_CONSTANTS::TRANSMISSION_ENCODER_POSITIONS[gear].first;
      gearEncoderPos =
          ROBOT_CONSTANTS::TRANSMISSION_ENCODER_POSITIONS[gear].second;
    }
  }

  void prev(void) {
    if (gear != 0) {
      gear--;
      currentGear = ROBOT_CONSTANTS::TRANSMISSION_ENCODER_POSITIONS[gear].first;
      gearEncoderPos =
          ROBOT_CONSTANTS::TRANSMISSION_ENCODER_POSITIONS[gear].second;
    }
  }
};

struct ShiftRequest {
  uint32_t requestedShiftTime = pros::millis();
  ShiftDirection direction;
  ShiftRequestState requestState;

  ShiftRequest(ShiftDirection d, ShiftRequestState s = ShiftRequestState::DONE)
      : direction(d), requestState(s) {}
};

class Drivetrain {
public:
  Drivetrain();
  int drive(int32_t throttle, int32_t turn, bool quickTurn = false,
            int32_t time = 0, int32_t prio = 0);
  int shiftUp();
  int shiftDown();
  std::array<std::array<uint32_t, 2>, 10> status();
  int updateDrivetrain(pros::Controller &c);
  inline void changeControlState(ControlState state) {
    drivetrainState = state;
  }
  int calibrateTransmissions();

private:
  std::pair<double, double> cheesyDrive(int throttleInput, int steeringInput,
                                        bool quickTurn);
  int updateTransmission(void);
  ControlState drivetrainState{ControlState::NONE};
  GearState transmissionState{GearState::FIRST};
  std::queue<ShiftRequest> transmissionQue{};
  Gear CurrentGear{};
  ShiftRequest currentRequestedMove{ShiftDirection::NONE};

  pros::MotorGroup leftDriveMotors{ROBOT_CONSTANTS::RIGHT_DRIVE_MOTORS};
  pros::MotorGroup rightDriveMotors{ROBOT_CONSTANTS::LEFT_DRIVE_MOTORS};

  pros::MotorGroup shiftMotors{ROBOT_CONSTANTS::TRANSMISSION_MOTORS};
};
