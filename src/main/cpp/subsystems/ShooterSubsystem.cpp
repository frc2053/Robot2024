// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/ShooterSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>

ShooterSubsystem::ShooterSubsystem() {
  ConfigureMotors();
  frc::SmartDashboard::PutData("Shooter Telemetry", this);
}

frc2::CommandPtr ShooterSubsystem::GoToSpeedCmd(std::function<double()> speed) {
  return frc2::cmd::Run([this, speed] { Set(speed()); }, {this})
      .FinallyDo([this] { Set(0); });
}

// This method will be called once per scheduler run
void ShooterSubsystem::Periodic() {
  ctre::phoenix6::BaseStatusSignal::RefreshAll(leftShooterVelSignal,
                                               rightShooterVelSignal);

  currentLeftVelocity =
      ConvertMotorVelToShooterVel(leftShooterVelSignal.GetValue());
  currentRightVelocity =
      ConvertMotorVelToShooterVel(rightShooterVelSignal.GetValue());
}

void ShooterSubsystem::SimulationPeriodic() {
  leftShooterSim.SetInput(
      frc::Vectord<1>{leftShooterSimState.GetMotorVoltage().value()});
  rightShooterSim.SetInput(
      frc::Vectord<1>{rightShooterSimState.GetMotorVoltage().value()});

  leftShooterSim.Update(20_ms);
  rightShooterSim.Update(20_ms);

  leftShooterSimState.SetRotorVelocity(
      ConvertShooterVelToMotorVel(leftShooterSim.GetAngularVelocity()));
  rightShooterSimState.SetRotorVelocity(
      ConvertShooterVelToMotorVel(rightShooterSim.GetAngularVelocity()));
}

void ShooterSubsystem::GoToVelocity(units::radians_per_second_t speed) {
  currentVelocitySetpoint = speed;
  units::radians_per_second_t motorSetpoint =
      ConvertShooterVelToMotorVel(speed);
  shooterLeftMotor.SetControl(
      velocityControl.WithVelocity(motorSetpoint).WithSlot(0));
  shooterRightMotor.SetControl(
      velocityControl.WithVelocity(motorSetpoint).WithSlot(0));
}

void ShooterSubsystem::Set(double speed) {
  shooterLeftMotor.SetControl(voltageController.WithOutput(speed * 12_V));
  shooterRightMotor.SetControl(voltageController.WithOutput(speed * -12_V));
}

units::radians_per_second_t ShooterSubsystem::GetLeftShooterCurrentVelocity() {
  return currentRightVelocity;
}

units::radians_per_second_t ShooterSubsystem::GetRightShooterCurrentVelocity() {
  return currentLeftVelocity;
}

bool ShooterSubsystem::IsShooterUpToSpeed() {
  return (units::math::abs(currentVelocitySetpoint -
                           GetLeftShooterCurrentVelocity()) <
          constants::shooter::SHOOTER_TOLERANCE) &&
         (units::math::abs(currentVelocitySetpoint -
                           GetRightShooterCurrentVelocity()) <
          constants::shooter::SHOOTER_TOLERANCE);
}

units::radians_per_second_t ShooterSubsystem::ConvertMotorVelToShooterVel(
    units::radians_per_second_t vel) {
  return vel * constants::shooter::SHOOTER_RATIO;
}

units::radians_per_second_t ShooterSubsystem::ConvertShooterVelToMotorVel(
    units::radians_per_second_t vel) {
  return vel / constants::shooter::SHOOTER_RATIO;
}

void ShooterSubsystem::ConfigureMotors() {
  ctre::phoenix6::configs::TalonFXConfiguration mainConfig;

  mainConfig.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Coast;

  mainConfig.Slot0.kP = currentGains.kP.to<double>();
  mainConfig.Slot0.kI = currentGains.kI.to<double>();
  mainConfig.Slot0.kD = currentGains.kD.to<double>();
  mainConfig.Slot0.kV = currentGains.kV.to<double>();
  mainConfig.Slot0.kA = currentGains.kA.to<double>();
  mainConfig.Slot0.kS = currentGains.kS.to<double>();

  shooterLeftMotor.GetConfigurator().Apply(mainConfig);
  shooterRightMotor.GetConfigurator().Apply(mainConfig);

  // Super fast refresh rate!
  ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
      1000_Hz, leftShooterVelSignal, rightShooterVelSignal);
  // Disable all other signals we dont care about
  shooterLeftMotor.OptimizeBusUtilization();
  shooterRightMotor.OptimizeBusUtilization();
}

void ShooterSubsystem::InitSendable(wpi::SendableBuilder& builder) {
  frc2::SubsystemBase::InitSendable(builder);
  builder.AddDoubleProperty(
      "Velocity Setpoint (RPM)",
      [this] {
        return currentVelocitySetpoint.convert<units::revolutions_per_minute>()
            .value();
      },
      [this](double newSetpointRpm) {
        GoToVelocity(units::revolutions_per_minute_t{newSetpointRpm});
      });
  builder.AddDoubleProperty(
      "Left Current Position (RPM)",
      [this] {
        return GetLeftShooterCurrentVelocity()
            .convert<units::revolutions_per_minute>()
            .value();
      },
      nullptr);
  builder.AddDoubleProperty(
      "Right Current Position (RPM)",
      [this] {
        return GetRightShooterCurrentVelocity()
            .convert<units::revolutions_per_minute>()
            .value();
      },
      nullptr);
  builder.AddDoubleProperty(
      "kP", [this] { return currentGains.kP.to<double>(); },
      [this](double newKp) {
        constants::shooter::ShooterGains newGains = GetGains();
        newGains.kP = units::radian_volt_kp_unit_t{newKp};
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kI", [this] { return currentGains.kI.to<double>(); },
      [this](double newKi) {
        constants::shooter::ShooterGains newGains = GetGains();
        newGains.kI = units::radian_volt_ki_unit_t{newKi};
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kD", [this] { return currentGains.kD.to<double>(); },
      [this](double newKd) {
        constants::shooter::ShooterGains newGains = GetGains();
        newGains.kD = units::radian_volt_kd_unit_t{newKd};
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kV", [this] { return currentGains.kV.to<double>(); },
      [this](double newKv) {
        constants::shooter::ShooterGains newGains = GetGains();
        newGains.kV = frc::DCMotor::radians_per_second_per_volt_t{newKv};
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kA", [this] { return currentGains.kA.to<double>(); },
      [this](double newKa) {
        constants::shooter::ShooterGains newGains = GetGains();
        newGains.kA = units::radial_ka_unit_t{newKa};
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kS", [this] { return currentGains.kS.to<double>(); },
      [this](double newKs) {
        constants::shooter::ShooterGains newGains = GetGains();
        newGains.kS = units::volt_t{newKs};
        SetGains(newGains);
      });
}

void ShooterSubsystem::SetGains(
    const constants::shooter::ShooterGains newGains) {
  currentGains = newGains;
  ctre::phoenix6::configs::Slot0Configs newConfig{};
  newConfig.kP = currentGains.kP.to<double>();
  newConfig.kI = currentGains.kI.to<double>();
  newConfig.kD = currentGains.kD.to<double>();
  newConfig.kV = currentGains.kV.to<double>();
  newConfig.kA = currentGains.kA.to<double>();
  newConfig.kS = currentGains.kS.to<double>();
  shooterLeftMotor.GetConfigurator().Apply(newConfig);
  shooterRightMotor.GetConfigurator().Apply(newConfig);
}

constants::shooter::ShooterGains ShooterSubsystem::GetGains() {
  return currentGains;
}