// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/ElevatorSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>

ElevatorSubsystem::ElevatorSubsystem() {
  ConfigureMotors();
  frc::SmartDashboard::PutData("Elevator Telemetry", this);
  frc::SmartDashboard::PutData("Elevator Vis", &mech2d);
}

frc2::CommandPtr ElevatorSubsystem::GoToHeightCommand(
    std::function<units::meter_t()> height) {
  return frc2::cmd::Sequence(
      frc2::cmd::RunOnce([height, this] { GoToHeight(height()); }, {this}),
      frc2::cmd::WaitUntil([this] { return IsElevatorAtSetpoint(); }));
};

// This method will be called once per scheduler run
void ElevatorSubsystem::Periodic() {
  // Refresh our signals from the motor controller
  ctre::phoenix6::BaseStatusSignal::RefreshAll(elevatorPositionSignal,
                                               elevatorVelocitySignal);

  units::turn_t motorPostion =
      ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          elevatorPositionSignal, elevatorVelocitySignal);

  currentPosition = ConvertMotorPositionToElevatorPositon(motorPostion);
  currentVelocity =
      ConvertMotorVelToElevatorVel(elevatorVelocitySignal.GetValue());

  elevatorMech2d->SetLength(currentPosition.value());
}

void ElevatorSubsystem::SimulationPeriodic() {
  // Our elevator simulator takes in the applied motor voltage as input in a 1x1
  // matrix
  elevatorSim.SetInput(
      frc::Vectord<1>{elevatorSimState.GetMotorVoltage().value()});
  // Advance the simulation by 20 milliseconds. This is the same update rate as
  // the periodic functions.
  elevatorSim.Update(20_ms);
  // Finally, update our simulated falcons encoders from the sim.
  elevatorSimState.SetRawRotorPosition(
      ConvertElevatorPositionToMotorPosition(elevatorSim.GetPosition()));
  elevatorSimState.SetRotorVelocity(
      ConvertElevatorVelToMotorVel(elevatorSim.GetVelocity()));
}

void ElevatorSubsystem::ConfigureMotors() {
  ctre::phoenix6::configs::TalonFXConfiguration mainConfig;

  mainConfig.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;

  mainConfig.Slot0.kP = currentGains.kP.to<double>();
  mainConfig.Slot0.kI = currentGains.kI.to<double>();
  mainConfig.Slot0.kD = currentGains.kD.to<double>();
  mainConfig.Slot0.kV = currentGains.kV.to<double>();
  mainConfig.Slot0.kA = currentGains.kA.to<double>();
  mainConfig.Slot0.kS = currentGains.kS.to<double>();
  mainConfig.Slot0.GravityType =
      ctre::phoenix6::signals::GravityTypeValue::Elevator_Static;
  mainConfig.Slot0.kG = currentGains.kG.to<double>();

  elevatorLeftMotor.GetConfigurator().Apply(mainConfig);
  elevatorRightMotor.GetConfigurator().Apply(mainConfig);

  // Because the other elevator motor is facing the opposite direction in our
  // imaginary elevator, we want to make sure it always follows the left motor
  // but in the opposite direction
  elevatorRightMotor.SetControl(ctre::phoenix6::controls::Follower{
      elevatorLeftMotor.GetDeviceID(), true});

  // Super fast refresh rate!
  ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
      1000_Hz, elevatorPositionSignal, elevatorVelocitySignal);
  // Disable all other signals we dont care about
  elevatorLeftMotor.OptimizeBusUtilization();
  elevatorRightMotor.OptimizeBusUtilization();
}

void ElevatorSubsystem::GoToHeight(units::meter_t height) {
  currentSetpoint = height;
  // This "withX" style of code is called the builder pattern.
  units::radian_t motorSetpoint =
      ConvertElevatorPositionToMotorPosition(height);
  elevatorLeftMotor.SetControl(positionControl.WithPosition(motorSetpoint)
                                   .WithEnableFOC(true)
                                   .WithSlot(0));
}

units::meter_t ElevatorSubsystem::GetCurrentHeight() {
  return currentPosition;
}

units::meter_t ElevatorSubsystem::ConvertMotorPositionToElevatorPositon(
    units::radian_t position) {
  units::radian_t drumRadialPosition =
      position / constants::elevator::ELEVATOR_RATIO;
  return units::meter_t{drumRadialPosition.value() *
                        constants::elevator::ELEVATOR_DRUM_RADIUS.value()};
}

units::meters_per_second_t ElevatorSubsystem::ConvertMotorVelToElevatorVel(
    units::radians_per_second_t vel) {
  units::radians_per_second_t drumRadialVelocity =
      vel / constants::elevator::ELEVATOR_RATIO;
  return units::meters_per_second_t{
      drumRadialVelocity.value() *
      constants::elevator::ELEVATOR_DRUM_RADIUS.value()};
}

units::radian_t ElevatorSubsystem::ConvertElevatorPositionToMotorPosition(
    units::meter_t position) {
  units::radian_t drumRadialPosition{
      position.value() / constants::elevator::ELEVATOR_DRUM_RADIUS.value()};
  return drumRadialPosition * constants::elevator::ELEVATOR_RATIO;
}

units::radians_per_second_t ElevatorSubsystem::ConvertElevatorVelToMotorVel(
    units::meters_per_second_t vel) {
  units::radians_per_second_t drumRadialVel{
      vel.value() / constants::elevator::ELEVATOR_DRUM_RADIUS.value()};
  return drumRadialVel * constants::elevator::ELEVATOR_RATIO;
}

bool ElevatorSubsystem::IsElevatorAtSetpoint() {
  return units::math::abs(currentSetpoint - GetCurrentHeight()) <
         constants::elevator::ELEVATOR_TOLERANCE;
}

void ElevatorSubsystem::SetGains(
    const constants::elevator::ElevatorGains& newGains) {
  currentGains = newGains;
  ctre::phoenix6::configs::Slot0Configs newConfig{};
  newConfig.kP = currentGains.kP.to<double>();
  newConfig.kI = currentGains.kI.to<double>();
  newConfig.kD = currentGains.kD.to<double>();
  newConfig.kV = currentGains.kV.to<double>();
  newConfig.kA = currentGains.kA.to<double>();
  newConfig.kS = currentGains.kS.to<double>();
  newConfig.GravityType =
      ctre::phoenix6::signals::GravityTypeValue::Elevator_Static;
  newConfig.kG = currentGains.kG.to<double>();
  elevatorLeftMotor.GetConfigurator().Apply(newConfig);
}

constants::elevator::ElevatorGains ElevatorSubsystem::GetGains() {
  return currentGains;
}

void ElevatorSubsystem::InitSendable(wpi::SendableBuilder& builder) {
  frc2::SubsystemBase::InitSendable(builder);
  builder.AddDoubleProperty(
      "Position Setpoint (ft)",
      [this] { return currentSetpoint.convert<units::feet>().value(); },
      [this](double newSetpointFt) {
        GoToHeight(units::foot_t{newSetpointFt});
      });
  builder.AddDoubleProperty(
      "Current Position (ft)",
      [this] { return GetCurrentHeight().convert<units::feet>().value(); },
      nullptr);
  builder.AddDoubleProperty(
      "Current Velocity (ft per sec)",
      [this] {
        return currentVelocity.convert<units::feet_per_second>().value();
      },
      nullptr);
  builder.AddDoubleProperty(
      "kP", [this] { return currentGains.kP.to<double>(); },
      [this](double newKp) {
        constants::elevator::ElevatorGains newGains = GetGains();
        newGains.kP = units::meter_volt_kp_unit_t{newKp};
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kI", [this] { return currentGains.kI.to<double>(); },
      [this](double newKi) {
        constants::elevator::ElevatorGains newGains = GetGains();
        newGains.kI = units::meter_volt_ki_unit_t{newKi};
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kD", [this] { return currentGains.kD.to<double>(); },
      [this](double newKd) {
        constants::elevator::ElevatorGains newGains = GetGains();
        newGains.kD = units::meter_volt_kd_unit_t{newKd};
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kV", [this] { return currentGains.kV.to<double>(); },
      [this](double newKv) {
        constants::elevator::ElevatorGains newGains = GetGains();
        newGains.kV = units::linear_kv_unit_t{newKv};
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kA", [this] { return currentGains.kA.to<double>(); },
      [this](double newKa) {
        constants::elevator::ElevatorGains newGains = GetGains();
        newGains.kA = units::linear_ka_unit_t{newKa};
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kS", [this] { return currentGains.kS.to<double>(); },
      [this](double newKs) {
        constants::elevator::ElevatorGains newGains = GetGains();
        newGains.kS = units::volt_t{newKs};
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kG", [this] { return currentGains.kG.to<double>(); },
      [this](double newKg) {
        constants::elevator::ElevatorGains newGains = GetGains();
        newGains.kG = units::volt_t{newKg};
        SetGains(newGains);
      });
}
