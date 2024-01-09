// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "RobotContainer.h"

#include <frc/MathUtil.h>
#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}

str::DeadbandAndSquareFunc RobotContainer::DeadbandAndSquare(
    std::function<double()> joystickValue) {
  return [joystickValue]() {
    double deadband = frc::ApplyDeadband<double>(joystickValue(), 0.2);
    return std::abs(deadband) * deadband;
  };
}
