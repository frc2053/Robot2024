// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/ClimberSubsystem.h"

ClimberSubsystem::ClimberSubsystem() = default;

// This method will be called once per scheduler run
void ClimberSubsystem::Periodic() {}

bool ClimberSubsystem::IsAtHeight() {
  return false;
}
