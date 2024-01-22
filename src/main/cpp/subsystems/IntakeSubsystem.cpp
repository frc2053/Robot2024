// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/IntakeSubsystem.h"

IntakeSubsystem::IntakeSubsystem() {
  ConfigureMotors();
}

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {}

void IntakeSubsystem::ConfigureMotors() {
  ctre::phoenix6::configs::TalonFXConfiguration mainConfig;

  mainConfig.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;

  intakeMotor.GetConfigurator().Apply(mainConfig);
}

frc2::CommandPtr IntakeSubsystem::SuckInNotes() {
  return frc2::cmd::RunEnd([this] { SetIntakeSpeed(1); },
                           [this] { SetIntakeSpeed(0); }, {this});
}

frc2::CommandPtr IntakeSubsystem::SpitOutNotes() {
  return frc2::cmd::RunEnd([this] { SetIntakeSpeed(-1); },
                           [this] { SetIntakeSpeed(0); }, {this});
}

void IntakeSubsystem::SetIntakeSpeed(double speed) {
  intakeMotor.Set(speed);
}
