// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/simulation/ElevatorSim.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include "Constants.h"

class ElevatorSubsystem : public frc2::SubsystemBase {
 public:
  ElevatorSubsystem();

  frc2::CommandPtr GoToHeightCommand(std::function<units::meter_t()> height);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void SimulationPeriodic() override;

  void GoToHeight(units::meter_t height);
  units::meter_t GetCurrentHeight();

  bool IsElevatorAtSetpoint();

 private:
  void ConfigureMotors();
  void InitSendable(wpi::SendableBuilder& builder) override;
  void SetGains(const constants::elevator::ElevatorGains& newGains);
  constants::elevator::ElevatorGains GetGains();

  units::meter_t ConvertMotorPositionToElevatorPositon(
      units::radian_t position);
  units::meters_per_second_t ConvertMotorVelToElevatorVel(
      units::radians_per_second_t vel);

  units::radian_t ConvertElevatorPositionToMotorPosition(
      units::meter_t position);
  units::radians_per_second_t ConvertElevatorVelToMotorVel(
      units::meters_per_second_t vel);

  ctre::phoenix6::hardware::TalonFX elevatorLeftMotor{
      constants::elevator::LEFT_ELEVATOR_CAN_ID};
  ctre::phoenix6::hardware::TalonFX elevatorRightMotor{
      constants::elevator::RIGHT_ELEVATOR_CAN_ID};

  ctre::phoenix6::controls::PositionVoltage positionControl{
      0_rad, 0_rad_per_s, true, 0_V, 0, false};
  ctre::phoenix6::StatusSignal<units::turn_t> elevatorPositionSignal{
      elevatorLeftMotor.GetPosition()};
  ctre::phoenix6::StatusSignal<units::turns_per_second_t>
      elevatorVelocitySignal{elevatorLeftMotor.GetVelocity()};

  units::meter_t currentSetpoint{0};
  units::meter_t currentPosition{0};
  units::meters_per_second_t currentVelocity{0};

  constants::elevator::ElevatorGains currentGains{};

  frc::DCMotor elevatorGearbox{frc::DCMotor::Falcon500FOC(2)};
  frc::sim::ElevatorSim elevatorSim{elevatorGearbox,
                                    constants::elevator::ELEVATOR_RATIO,
                                    constants::elevator::CARRIAGE_MASS,
                                    constants::elevator::ELEVATOR_DRUM_RADIUS,
                                    0_m,
                                    constants::elevator::ELEVATOR_MAX_HEIGHT,
                                    true,
                                    0_m,
                                    {0.005}};

  ctre::phoenix6::sim::TalonFXSimState& elevatorSimState{
      elevatorLeftMotor.GetSimState()};

  // Create a Mechanism2d display of an elevator
  frc::Mechanism2d mech2d{10_in / 1_m, 73_in / 1_m};
  frc::MechanismRoot2d* elevatorRoot =
      mech2d.GetRoot("Elevator Root", 5_in / 1_m, 0.0_in / 1_m);
  frc::MechanismLigament2d* elevatorMech2d =
      elevatorRoot->Append<frc::MechanismLigament2d>(
          "Elevator", elevatorSim.GetPosition().value(), 90_deg);
};
