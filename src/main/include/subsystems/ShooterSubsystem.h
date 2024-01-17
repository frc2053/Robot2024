// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/simulation/FlywheelSim.h>
#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include "Constants.h"

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  ShooterSubsystem();

  frc2::CommandPtr GoToSpeedCmd(std::function<double()> speed);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void Set(double speed);

  void GoToVelocity(units::radians_per_second_t speed);
  units::radians_per_second_t GetLeftShooterCurrentVelocity();
  units::radians_per_second_t GetRightShooterCurrentVelocity();

 private:
  void ConfigureMotors();
  void InitSendable(wpi::SendableBuilder& builder) override;
  void SetGains(const constants::shooter::ShooterGains newGains);
  constants::shooter::ShooterGains GetGains();

  void SimulationPeriodic() override;

  bool IsShooterUpToSpeed();

  units::radians_per_second_t ConvertMotorVelToShooterVel(
      units::radians_per_second_t vel);
  units::radians_per_second_t ConvertShooterVelToMotorVel(
      units::radians_per_second_t vel);

  ctre::phoenix6::hardware::TalonFX shooterLeftMotor{
      constants::shooter::LEFT_SHOOTER_CAN_ID};
  ctre::phoenix6::hardware::TalonFX shooterRightMotor{
      constants::shooter::RIGHT_SHOOTER_CAN_ID};

  ctre::phoenix6::controls::VelocityTorqueCurrentFOC velocityControl{
      0_rad_per_s};
  ctre::phoenix6::controls::VoltageOut voltageController{0_V};

  ctre::phoenix6::StatusSignal<units::turns_per_second_t> leftShooterVelSignal{
      shooterLeftMotor.GetVelocity()};
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> rightShooterVelSignal{
      shooterRightMotor.GetVelocity()};

  units::radians_per_second_t currentVelocitySetpoint{0};
  units::radians_per_second_t currentLeftVelocity{0};
  units::radians_per_second_t currentRightVelocity{0};

  constants::shooter::ShooterGains currentGains{};

  frc::DCMotor shooterGearbox{frc::DCMotor::Falcon500FOC(1)};
  frc::sim::FlywheelSim leftShooterSim{shooterGearbox,
                                       constants::shooter::SHOOTER_RATIO,
                                       constants::shooter::SHOOTER_MOI};

  frc::sim::FlywheelSim rightShooterSim{shooterGearbox,
                                        constants::shooter::SHOOTER_RATIO,
                                        constants::shooter::SHOOTER_MOI};

  ctre::phoenix6::sim::TalonFXSimState& leftShooterSimState{
      shooterLeftMotor.GetSimState()};

  ctre::phoenix6::sim::TalonFXSimState& rightShooterSimState{
      shooterRightMotor.GetSimState()};
};
