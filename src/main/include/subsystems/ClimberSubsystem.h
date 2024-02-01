// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include "Constants.h"

class ClimberSubsystem : public frc2::SubsystemBase {
 public:
  ClimberSubsystem();

  void ConfigureMotor();

  void Periodic() override;
  bool IsAtHeight();
  void SetClimbHeight(units::meter_t newSetpoint);
  void GoDown();
  units::meter_t GetClimberHeight();

 private:
  rev::CANSparkMax mainClimbMotor{constants::climber::MAIN_CLIMBER_CAN_ID,
                                  rev::CANSparkLowLevel::MotorType::kBrushless};

  rev::CANSparkMax followClimbMotor{
      constants::climber::FOLLOW_CLIMBER_CAN_ID,
      rev::CANSparkLowLevel::MotorType::kBrushless};

  rev::SparkPIDController m_pidController = mainClimbMotor.GetPIDController();
  rev::SparkRelativeEncoder m_encoder =
      mainClimbMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

  void InitSendable(wpi::SendableBuilder& builder) override;
  void SetGains(const constants::climber::ClimberGains newGains);
  constants::climber::ClimberGains GetGains();

  constants::climber::ClimberGains currentGains = constants::climber::GAINS;
  units::meter_t currentSetpoint;
};
