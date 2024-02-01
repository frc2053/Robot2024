// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/ClimberSubsystem.h"

#include "frc/smartdashboard/SmartDashboard.h"
#include "wpi/sendable/SendableBuilder.h"

ClimberSubsystem::ClimberSubsystem() = default;

void ClimberSubsystem::ConfigureMotor() {
  mainClimbMotor.RestoreFactoryDefaults();
  followClimbMotor.RestoreFactoryDefaults();

  m_encoder.SetPositionConversionFactor(
      constants::climber::CLIMBER_SPOOL_RADIUS.value() *
      constants::climber::CLIMBER_RATIO);
  m_encoder.SetVelocityConversionFactor(
      constants::climber::CLIMBER_SPOOL_RADIUS.value() *
      constants::climber::CLIMBER_RATIO);

  // mainClimbMotor.SetP();
  // mainClimbMotor.SetI(kI);
  // mainClimbMotor.SetD(kD);
  // mainClimbMotor.SetIZone(kIz);
  // mainClimbMotor.SetFF(kFF);
  // mainClimbMotor.SetOutputRange(kMinOutput, kMaxOutput);

  followClimbMotor.Follow(mainClimbMotor, true);
}

// This method will be called once per scheduler run
void ClimberSubsystem::Periodic() {}

units::meter_t ClimberSubsystem::GetClimberHeight() {
  return units::meter_t{m_encoder.GetPosition()};
}

bool ClimberSubsystem::IsAtHeight() {
  if (units::math::abs(GetClimberHeight() - currentSetpoint) <
      constants::climber::CLIMBER_TOLERANCE) {
    return true;
  } else {
    return false;
  }
}

void ClimberSubsystem::SetClimbHeight(units::meter_t newSetpoint) {
  currentSetpoint = newSetpoint;
  m_pidController.SetReference(currentSetpoint.value(),
                               rev::ControlType::kPosition, 0, 0,
                               rev::CANPIDController::ArbFFUnits::kVoltage);
}

void ClimberSubsystem::InitSendable(wpi::SendableBuilder& builder) {
  frc2::SubsystemBase::InitSendable(builder);
  builder.AddDoubleProperty(
      "Height Goal (Inches)",
      [this] { return currentSetpoint.convert<units::inches>().value(); },
      [this](double newGoalInches) {
        SetClimbHeight(units::inch_t{newGoalInches});
      });
  builder.AddDoubleProperty(
      "Height Position (Inches)",
      [this] { return GetClimberHeight().convert<units::inches>().value(); },
      nullptr);
  builder.AddDoubleProperty(
      "kP", [this] { return currentGains.kP.to<double>(); },
      [this](double newKp) {
        constants::climber::ClimberGains newGains = GetGains();
        newGains.kP = units::radian_volt_kp_unit_t{newKp};
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kI", [this] { return currentGains.kI.to<double>(); },
      [this](double newKi) {
        constants::climber::ClimberGains newGains = GetGains();
        newGains.kI = units::radian_volt_ki_unit_t{newKi};
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kD", [this] { return currentGains.kD.to<double>(); },
      [this](double newKd) {
        constants::climber::ClimberGains newGains = GetGains();
        newGains.kD = units::radian_volt_kd_unit_t{newKd};
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kV", [this] { return currentGains.kV.to<double>(); },
      [this](double newKv) {
        constants::climber::ClimberGains newGains = GetGains();
        newGains.kV = units::unit_t<frc::ElevatorFeedforward::kv_unit>{newKv};
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kA", [this] { return currentGains.kA.to<double>(); },
      [this](double newKa) {
        constants::climber::ClimberGains newGains = GetGains();
        newGains.kA = units::unit_t<frc::ElevatorFeedforward::ka_unit>{newKa};
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kS", [this] { return currentGains.kS.to<double>(); },
      [this](double newKs) {
        constants::climber::ClimberGains newGains = GetGains();
        newGains.kS = units::volt_t{newKs};
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kG", [this] { return currentGains.kG.to<double>(); },
      [this](double newKg) {
        constants::climber::ClimberGains newGains = GetGains();
        newGains.kG = units::volt_t{newKg};
        SetGains(newGains);
      });
}

void ClimberSubsystem::SetGains(
    const constants::climber::ClimberGains newGains) {
  currentGains = newGains;
  // pivotFeedfoward = frc::ArmFeedforward{newGains.kS, newGains.kG,
  // newGains.kV, newGains.kA};
  m_pidController.SetP(newGains.kP.value());
  m_pidController.SetP(newGains.kI.value());
  m_pidController.SetP(newGains.kD.value());
}

constants::climber::ClimberGains ClimberSubsystem::GetGains() {
  return currentGains;
}
