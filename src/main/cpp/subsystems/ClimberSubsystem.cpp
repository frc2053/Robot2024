// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/ClimberSubsystem.h"

ClimberSubsystem::ClimberSubsystem() = default;

// This method will be called once per scheduler run
void ClimberSubsystem::Periodic() {}

units::meter_t ConvertMotorPositionToClimberPositon(units::radian_t position) {
  units::meter_t climbpos = 1_in;
  return climbpos
}

units::meters_per_second_t ConvertMotorVelToClimberVel(
    units::radians_per_second_t vel) {}

bool ClimberSubsystem::IsAtHeight() {
  if (currentPosition > currentSetpoint + 1_in ||
      currentPosition < currentSetpoint - 1_in)
    return false;

  else
    return true;
}

void SetClimbHeight() {}
