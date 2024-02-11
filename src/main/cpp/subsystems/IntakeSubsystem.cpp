// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/IntakeSubsystem.h"

#include <frc/RobotBase.h>

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

frc2::CommandPtr IntakeSubsystem::SuckInNotes() {
  return frc2::cmd::RunEnd([this] { SetIntakeSpeed(1); },
                           [this] { SetIntakeSpeed(0); }, {this})
      .BeforeStarting(
          [this] {
            if (frc::RobotBase::IsSimulation()) {
              if (SeesNote()) {
                intakeSensor.SetDistance(12_in);
              } else {
                intakeSensor.SetDistance(0_in);
              }
            }
          },
          {this});
}

frc2::CommandPtr IntakeSubsystem::SpitOutNotes() {
  return frc2::cmd::RunEnd([this] { SetIntakeSpeed(-1); },
                           [this] { SetIntakeSpeed(0); }, {this});
}

frc2::CommandPtr IntakeSubsystem::SuckInUntilNoteIsSeen() {
  return SuckInNotes().Until([this] { return SeesNote(); });
}

bool IntakeSubsystem::SeesNote() {
  return units::millimeter_t{intakeSensor.GetRange()} <=
         constants::intake::INTAKE_SENSOR_DISTANCE;
}

void IntakeSubsystem::SetIntakeSpeed(double speed) {
  intakeMotor.Set(speed);
}
