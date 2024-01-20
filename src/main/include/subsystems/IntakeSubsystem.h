// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include "Constants.h"

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem();

  frc2::CommandPtr SuckInNotes() {
    return frc2::cmd::RunEnd([this] { SetIntakeSpeed(1); },
                             [this] { SetIntakeSpeed(0); }, {this});
  }

  frc2::CommandPtr SpitOutNotes() {
    return frc2::cmd::RunEnd([this] { SetIntakeSpeed(-1); },
                             [this] { SetIntakeSpeed(0); }, {this});
  }

  void Periodic() override;

 private:
  ctre::phoenix6::hardware::TalonFX intakeMotor{
      constants::intake::INTAKE_CAN_ID};

  void SetIntakeSpeed(double speed) { intakeMotor.Set(speed); }
};
