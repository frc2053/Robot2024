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

  void Periodic() override;
  bool IsAtHeight();
  void SetClimbHeight();
  void GoDown();

 private:
  rev::CANSparkMax mainClimbMotor{constants::climber::MAIN_CLIMBER_CAN_ID,
                                  rev::CANSparkLowLevel::MotorType::kBrushless};

  rev::CANSparkMax followClimbMotor{
      constants::climber::FOLLOW_CLIMBER_CAN_ID,
      rev::CANSparkLowLevel::MotorType::kBrushless};

  units::meter_t currentSetpoint{0};
  units::meter_t currentPosition{0};
  units::meters_per_second_t currentVelocity{0};
};
