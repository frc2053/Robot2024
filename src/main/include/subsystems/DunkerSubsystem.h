// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include "Constants.h"

class DunkerSubsystem : public frc2::SubsystemBase {
 public:
  DunkerSubsystem();

  void Periodic() override;
  frc2::CommandPtr PivotDunkNotesOut();
  frc2::CommandPtr PivotDunkNotesIn();
  frc2::CommandPtr DunkTheNotes();
  frc2::CommandPtr JammedDunkNotes();

 private:
  void ConfigureMotors();

  ctre::phoenix6::hardware::TalonFX dunkPivotMotor{
      constants::dunker::PIVOT_DUNKER_CAN_ID};

  ctre::phoenix6::controls::MotionMagicVoltage positionSetter{0_rad};

  ctre::phoenix6::StatusSignal<units::turn_t> dunkPivotPositionSig{
      dunkPivotMotor.GetPosition()};

  rev::CANSparkMax dunkMotor{constants::dunker::DUNKER_CAN_ID,
                             rev::CANSparkLowLevel::MotorType::kBrushless};

  units::radian_t currentPivotPos = 0_rad;
  units::radian_t currentPivotSetpoint = 0_rad;

  void SetDunkSpeed(double speed) { dunkMotor.SetVoltage(speed * 12_V); }

  void SetPivotAngle(units::radian_t angleSetPoint) {
    currentPivotSetpoint = angleSetPoint;
    dunkPivotMotor.SetControl(positionSetter.WithPosition(angleSetPoint));
  }

  bool IsPivotSetPoint() {
    return units::math::abs(currentPivotPos - currentPivotSetpoint) <=
           constants::dunker::DUNKER_PIVOT_ANGLE_TOLERANCE;
  }
};
