#include "Drivetrain.hpp"
#include <cmath>

Drivetrain::Drivetrain() {
  leftDriveMotors.set_brake_mode_all(ROBOT_CONSTANTS::DRIVE_BRAKE_MODE);
  leftDriveMotors.set_gearing_all(ROBOT_CONSTANTS::DRIVE_GEARING);
  rightDriveMotors.set_gearing_all(ROBOT_CONSTANTS::DRIVE_GEARING);
  rightDriveMotors.set_brake_mode_all(ROBOT_CONSTANTS::DRIVE_BRAKE_MODE);

  shiftMotors.set_brake_mode_all(ROBOT_CONSTANTS::SHIFT_BRAKE_MODE);
  shiftMotors.set_gearing_all(ROBOT_CONSTANTS::SHIFT_GEARINNG);
  shiftMotors.set_encoder_units_all(pros::MotorUnits::degrees);
}

int Drivetrain::shiftUp() {
  transmissionQue.push(
      ShiftRequest(ShiftDirection::UP, ShiftRequestState::REQUESTED));
  return 0;
}

int Drivetrain::shiftDown() {
  transmissionQue.push(
      ShiftRequest(ShiftDirection::DOWN, ShiftRequestState::REQUESTED));
  return 0;
}

int Drivetrain::updateTransmission(void) {
  if (!transmissionQue.empty()) {
    ShiftRequest &currentRequest = transmissionQue.front();
    switch (currentRequest.requestState) {
    case ShiftRequestState::DONE:
      currentRequestedMove = currentRequest;
      transmissionQue.pop();
      return 0;
    case ShiftRequestState::REQUESTED:
      currentRequest.requestedShiftTime = pros::millis();
      if (currentRequest.direction == ShiftDirection::UP) {
        CurrentGear.next();
      } else {
        CurrentGear.prev();
      }
      shiftMotors.move_absolute(CurrentGear.gearEncoderPos,
                                ROBOT_CONSTANTS::C_TRANSMISSION_MAX_SPEED);
      currentRequest.requestState = ShiftRequestState::SHIFTING;
      break;
    case ShiftRequestState::SHIFTING:
      if ((std::abs(shiftMotors.get_position_all().at(0) -
                    CurrentGear.gearEncoderPos) < 3.0) &&
          (std::abs(shiftMotors.get_position_all().at(1) -
                    CurrentGear.gearEncoderPos) < 3.0)) {
        currentRequest.requestState = ShiftRequestState::DONE;
      } else if ((pros::millis() - currentRequest.requestedShiftTime) >
                 ROBOT_CONSTANTS::C_TRANSMISSION_SHIFT_TIME_MS) {
        currentRequest.requestState = ShiftRequestState::NONE;
      }
      break;
    case ShiftRequestState::NONE:
      currentRequest.requestedShiftTime = pros::millis();
      if (currentRequest.direction == ShiftDirection::UP) {
        CurrentGear.prev();
      } else {
        CurrentGear.next();
      }
      shiftMotors.move_absolute(CurrentGear.gearEncoderPos,
                                ROBOT_CONSTANTS::C_TRANSMISSION_MAX_SPEED);
      currentRequest.requestState = ShiftRequestState::SHIFTING;
      break;
    }
    currentRequestedMove = currentRequest;
  }
  return 0;
}

int Drivetrain::updateDrivetrain(pros::Controller &c) {
  if (c.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
    shiftDown();
  else if (c.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1))
    shiftUp();
  updateTransmission();
  if (currentRequestedMove.requestState != ShiftRequestState::DONE) {
    leftDriveMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    rightDriveMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    leftDriveMotors.brake();
    rightDriveMotors.brake();
  } else {
    leftDriveMotors.set_brake_mode_all(ROBOT_CONSTANTS::DRIVE_BRAKE_MODE);
    rightDriveMotors.set_brake_mode_all(ROBOT_CONSTANTS::DRIVE_BRAKE_MODE);
    drive(c.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
          c.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X),
          c.get_digital(pros::E_CONTROLLER_DIGITAL_X));
  }
  return 0;
}

int Drivetrain::drive(int32_t throttle, int32_t turn, bool quickTurn,
                      int32_t time, int32_t prio) {
  // // Applied Deadband
  // if (abs(throttle) <= ROBOT_CONSTANTS::C_JOYSTICK_DEADBAND)
  //   throttle = 0;
  // if (abs(turn) <= ROBOT_CONSTANTS::C_JOYSTICK_DEADBAND)
  //   turn = 0;
  // std::pair<double, double> driveVals;

  // driveVals = cheesyDrive(throttle, turn, quickTurn);
  // leftDriveMotors.move_voltage(static_cast<int32_t>(
  //     driveVals.first * ROBOT_CONSTANTS::C_MAX_DRIVE_MOTOR_VOLTAGE_MV));
  // rightDriveMotors.move_voltage(static_cast<int32_t>(
  //     driveVals.first * ROBOT_CONSTANTS::C_MAX_DRIVE_MOTOR_VOLTAGE_MV));
  return 0;
}

std::array<std::array<uint32_t, 2>, 10> Drivetrain::status() {
  std::array<std::array<uint32_t, 2>, 10> returnArray = {{{0, 0},
                                                          {0, 0},
                                                          {0, 0},
                                                          {0, 0},
                                                          {0, 0},
                                                          {0, 0},
                                                          {0, 0},
                                                          {0, 0},
                                                          {0, 0},
                                                          {0, 0}}};
  return returnArray;
}

int Drivetrain::calibrateTransmissions(void) {
  uint32_t timeoutCounter = pros::millis();
  bool leftCalibrated = false;
  bool rightCalibrated = false;
  while (!(rightCalibrated && leftCalibrated) &&
         (pros::millis() <
          (timeoutCounter + ROBOT_CONSTANTS::C_TRANSMISSION_ZERO_TIMEOUT_MS))) {
    shiftMotors.move_velocity(50);
    pros::delay(500);
    if (std::abs(shiftMotors.get_actual_velocity(0)) <
            ROBOT_CONSTANTS::C_TRANSMISSION_HOMING_ZERO_SPEED &&
        shiftMotors.get_current_draw(0) >
            ROBOT_CONSTANTS::C_TRANSMISSION_ZERO_CURRENT_MA) {
      leftCalibrated = true;
      shiftMotors.set_current_limit(0, 0);
      shiftMotors.tare_position(0);
    }
    if (std::abs(shiftMotors.get_actual_velocity(1)) <
            ROBOT_CONSTANTS::C_TRANSMISSION_HOMING_ZERO_SPEED &&
        shiftMotors.get_current_draw(1) >
            ROBOT_CONSTANTS::C_TRANSMISSION_ZERO_CURRENT_MA) {
      rightCalibrated = true;
      shiftMotors.set_current_limit(0, 1);
      shiftMotors.tare_position(1);
    }
  }
  shiftMotors.move_velocity(0);
  shiftMotors.set_current_limit_all(2500);

  if (!(leftCalibrated & rightCalibrated)) {
    return 1;
  }
  return 0;
}

std::pair<double, double> cheesyDrive(int throttleInput, int steeringInput,
                                      bool quickTurn) {
  double throttle = std::clamp(throttleInput, -127, 127) / 127.0;
  double steering = std::clamp(steeringInput, -127, 127) / 127.0;

  double turn = quickTurn ? steering : std::abs(throttle) * steering;

  double left = throttle + turn;
  double right = throttle - turn;

  double scale = std::max({1.0, std::abs(left), std::abs(right)});
  return {left / scale, right / scale};
}
