// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/IntakeSubsystem.h"

IntakeSubsystem::IntakeSubsystem() {
  ConfigureMotors();

  intakeSensor.SetRangingMode(frc::TimeOfFlight::RangingMode::kShort, 24);
}

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {}

void IntakeSubsystem::ConfigureMotors() {
  ctre::phoenix6::configs::TalonFXConfiguration mainConfig;

  mainConfig.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;

  intakeMotor.GetConfigurator().Apply(mainConfig);
}

frc2::CommandPtr IntakeSubsystem::FeedIntake() {
  return frc2::cmd::RunEnd([this] { SetIntakeSpeed(1); },
                           [this] { SetIntakeSpeed(0); }, {this});
}

frc2::CommandPtr IntakeSubsystem::IntakeJammed() {
  return frc2::cmd::RunEnd([this] { SetIntakeSpeed(-1); },
                           [this] { SetIntakeSpeed(0); }, {this});
}

frc2::CommandPtr IntakeSubsystem::SuckInUntilNoteIsSeen() {
  return FeedIntake().Until([this] { return SeesNote(); });
}

bool IntakeSubsystem::SeesNote() {
  return units::millimeter_t{intakeSensor.GetRange()} <=
         constants::intake::INTAKE_SENSOR_DISTANCE;
}

void IntakeSubsystem::SetIntakeSpeed(double speed) {
  intakeMotor.Set(speed);
}
