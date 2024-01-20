// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include <functional>

#include "auto/Autos.h"
#include "subsystems/DrivebaseSubsystem.h"
#include "subsystems/DunkerSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"

namespace str {
using DeadbandAndSquareFunc = std::function<double()>;
}  // namespace str

class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();
  DrivebaseSubsystem& GetDrivebaseSubsystem();
  ShooterSubsystem& GetShooterSubsystem();

 private:
  void ConfigureBindings();
  frc2::CommandXboxController driverController{0};
  frc2::CommandXboxController operatorController{1};

  DrivebaseSubsystem driveSub;
  ShooterSubsystem shooterSub;
  IntakeSubsystem intakeSub;
  DunkerSubsystem dunkSub;

  autos::Autos autos{driveSub, shooterSub};

  frc2::CommandPtr characterizeSteerCmd = driveSub.CharacterizeSteerMotors([] {
    return frc::SmartDashboard::GetBoolean("Drivebase/DoneWithStep", false);
  });
  frc2::CommandPtr characterizeDriveCmd = driveSub.CharacterizeDriveMotors(
      [this] { return driverController.GetStartButtonPressed(); });
  frc2::CommandPtr selfTestCmd = driveSub.SelfTest();
  frc2::CommandPtr measureWheelCmd = driveSub.MeasureWheelDiam([] {
    return frc::SmartDashboard::GetBoolean("Drivebase/DoneWithStep", false);
  });
  frc2::CommandPtr tuneSteerCmd = driveSub.TuneSteerPID([] {
    return frc::SmartDashboard::GetBoolean("Drivebase/DoneWithStep", false);
  });
  frc2::CommandPtr tuneDriveCmd = driveSub.TuneDrivePID([] {
    return frc::SmartDashboard::GetBoolean("Drivebase/DoneWithStep", false);
  });
  frc2::CommandPtr resetPositionCmd =
      driveSub.ResetPosition([] { return frc::Pose2d{}; });
  frc2::CommandPtr tunePathPidCmd = driveSub.TunePathPid();
  frc2::CommandPtr donePathTuningCmd = driveSub.DoneTuningPathPids();
  frc2::CommandPtr zeroYawCMD = driveSub.ZeroYawCMD();

  str::DeadbandAndSquareFunc DeadbandAndSquare(
      std::function<double()> joystickValue);
};
