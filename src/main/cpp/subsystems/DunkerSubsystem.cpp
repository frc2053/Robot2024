// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/DunkerSubsystem.h"

DunkerSubsystem::DunkerSubsystem() {
  ConfigureMotors();
}

void DunkerSubsystem::ConfigureMotors() {
  dunkMotor.RestoreFactoryDefaults();
  dunkMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
  dunkMotor.SetSmartCurrentLimit(20);
  dunkMotor.BurnFlash();
}
