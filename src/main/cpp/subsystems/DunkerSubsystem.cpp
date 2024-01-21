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
  if(dunkMotor.BurnFlash() == rev::REVLibError::kOk) {
    fmt::print("Successfully configured dunk motor!\n");
  }
  else {
    fmt::print("ERROR: Unable to configure dunk motor!\n");
  }
}

void DunkerSubsystem::Periodic() {
  ctre::phoenix6::BaseStatusSignal::RefreshAll(dunkPivotPositionSig);

  currentPivotPos = dunkPivotPositionSig.GetValue();
}

frc2::CommandPtr DunkerSubsystem::PivotDunkNotesOut() {
  return frc2::cmd::Sequence(
      frc2::cmd::RunOnce(
          [this] { SetPivotAngle(constants::dunker::DUNKER_OUT_ANGLE); },
          {this}),
      frc2::cmd::WaitUntil([this] { return IsPivotSetPoint(); }));
}

frc2::CommandPtr DunkerSubsystem::PivotDunkNotesIn() {
  return frc2::cmd::Sequence(
      frc2::cmd::RunOnce(
          [this] { SetPivotAngle(constants::dunker::DUNKER_IN_ANGLE); },
          {this}),
      frc2::cmd::WaitUntil([this] { return IsPivotSetPoint(); }));
}

frc2::CommandPtr DunkerSubsystem::DunkTheNotes() {
  return frc2::cmd::RunEnd(
      [this] {
        SetDunkSpeed(1);
      },
      [this] { SetDunkSpeed(0); }, {this});
}

frc2::CommandPtr DunkerSubsystem::JammedDunkNotes() {
  return frc2::cmd::RunEnd(
      [this] {
        SetDunkSpeed(-1);
      },
      [this] { SetDunkSpeed(0); }, {this});
}

