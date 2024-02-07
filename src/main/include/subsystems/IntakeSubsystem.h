// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>

#include <TimeOfFlight.h>
#include <ctre/phoenix6/TalonFX.hpp>

#include "Constants.h"

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem();

  frc2::CommandPtr FeedIntake();
  frc2::CommandPtr IntakeJammed();
  frc2::CommandPtr SuckInUntilNoteIsSeen();

  void Periodic() override;

 private:
  ctre::phoenix6::hardware::TalonFX intakeMotor{
      constants::intake::INTAKE_CAN_ID};

  frc::TimeOfFlight intakeSensor{constants::intake::INTAKE_TOF_SENSOR};

  void ConfigureMotors();
  void SetIntakeSpeed(double speed);
  bool SeesNote();
};
