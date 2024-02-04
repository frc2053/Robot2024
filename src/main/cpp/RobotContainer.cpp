// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "RobotContainer.h"

#include <frc/DriverStation.h>
#include <frc/MathUtil.h>
#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  operatorController.A().WhileTrue(shooterSub.GoToSpeedCmd([this] {
    return frc::ApplyDeadband<double>(operatorController.GetLeftY(), 0.2);
  }));

  operatorController.B().WhileTrue(IntakeNote());

  operatorController.X().WhileTrue(intakeSub.SpitOutNotes());

  operatorController.Y().WhileTrue(dunkSub.PivotDunkNotesOut());
  operatorController.Y().OnFalse(dunkSub.PivotDunkNotesIn());

  operatorController.RightBumper().WhileTrue(dunkSub.DunkTheNotes());
  operatorController.LeftBumper().WhileTrue(dunkSub.JammedDunkNotes());

  operatorController.RightTrigger().WhileTrue(SpinUpShooter());
  operatorController.RightTrigger().OnFalse(NotUsingShooter());

  // operatorController.Start().WhileTrue(
  //     shooterSub.SysIdQuasistatic(frc2::sysid::Direction::kForward));
  // operatorController.Back().WhileTrue(
  //     shooterSub.SysIdQuasistatic(frc2::sysid::Direction::kReverse));
  // operatorController.RightTrigger().WhileTrue(
  //     shooterSub.SysIdDynamic(frc2::sysid::Direction::kForward));
  // operatorController.LeftTrigger().WhileTrue(
  //     shooterSub.SysIdDynamic(frc2::sysid::Direction::kReverse));

  driverController.RightBumper().WhileTrue(
      frc2::cmd::Defer(GetAStarCmd(), {&driveSub})
          .Unless([this] { return driveSub.InSafeZone(); })
          .AndThen(driveSub.GoToPose([this] {
            frc::Pose2d closestPoint =
                driveSub.CalculateClosestGoodShooterPoint();
            return closestPoint;
          }))
          .AndThen(driveSub.MoveAlongArc(
              [this] {
                return frc::ApplyDeadband<double>(-driverController.GetLeftX(),
                                                  .2);
              },
              [this] {
                return driveSub.CalculateClosestGoodShooterPoint()
                    .Rotation()
                    .Radians();
              })));

  driveSub.SetDefaultCommand(driveSub.DriveFactory(
      DeadbandAndSquare([this] { return -driverController.GetLeftY(); }),
      DeadbandAndSquare([this] { return -driverController.GetLeftX(); }),
      DeadbandAndSquare([this] { return -driverController.GetRightX(); })));

  driverController.Y().OnTrue(driveSub.TurnToAngleFactory(
      DeadbandAndSquare([this] { return -driverController.GetLeftY(); }),
      DeadbandAndSquare([this] { return -driverController.GetLeftX(); }),
      [this] {
        return frc::TrapezoidProfile<units::radians>::State{
            ShouldFlipAngleForDriver(0_deg), 0_deg_per_s};
      },
      [this] { return std::abs(driverController.GetRightX()) > 0.2; }));

  driverController.X().OnTrue(driveSub.TurnToAngleFactory(
      DeadbandAndSquare([this] { return -driverController.GetLeftY(); }),
      DeadbandAndSquare([this] { return -driverController.GetLeftX(); }),
      [this] {
        return frc::TrapezoidProfile<units::radians>::State{
            ShouldFlipAngleForDriver(90_deg), 0_deg_per_s};
      },
      [this] { return std::abs(driverController.GetRightX()) > 0.2; }));

  driverController.B().OnTrue(driveSub.TurnToAngleFactory(
      DeadbandAndSquare([this] { return -driverController.GetLeftY(); }),
      DeadbandAndSquare([this] { return -driverController.GetLeftX(); }),
      [this] {
        return frc::TrapezoidProfile<units::radians>::State{
            ShouldFlipAngleForDriver(-90_deg), 0_deg_per_s};
      },
      [this] { return std::abs(driverController.GetRightX()) > 0.2; }));

  driverController.A().OnTrue(driveSub.TurnToAngleFactory(
      DeadbandAndSquare([this] { return -driverController.GetLeftY(); }),
      DeadbandAndSquare([this] { return -driverController.GetLeftX(); }),
      [this] {
        return frc::TrapezoidProfile<units::radians>::State{
            ShouldFlipAngleForDriver(180_deg), 0_deg_per_s};
      },
      [this] { return std::abs(driverController.GetRightX()) > 0.2; }));

  frc::SmartDashboard::PutBoolean("Drivebase/DoneWithStep", false);

  frc::SmartDashboard::PutData("Drivebase/SelfTestCmd", selfTestCmd.get());

  frc::SmartDashboard::PutData("Drivebase/MeasureWheelCmd",
                               measureWheelCmd.get());

  frc::SmartDashboard::PutData("Drivebase/TuneSteerCmd", tuneSteerCmd.get());

  frc::SmartDashboard::PutData("Drivebase/ZeroYawCMD", zeroYawCMD.get());

  frc::SmartDashboard::PutData("Drivebase/TuneDriveCmd", tuneDriveCmd.get());

  frc::SmartDashboard::PutData("Drivebase/ResetPosition",
                               resetPositionCmd.get());

  frc::SmartDashboard::PutData("Drivebase/PathTuningCmd", tunePathPidCmd.get());

  frc::SmartDashboard::PutData("Drivebase/DonePathTuningCmd",
                               donePathTuningCmd.get());
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return autos.GetSelectedAutoCmd.get();
}

frc2::CommandPtr RobotContainer::SpinUpShooter() {
  return frc2::cmd::Sequence(ledSub.SetBothToBlinkRed(),
                             shooterSub.GoToVelocityCmd([] {
                               return constants::shooter::SHOOTER_SPEED;
                             }),
                             ledSub.SetBothToSolidGreen(), RumbleOperator());
}

frc2::CommandPtr RobotContainer::NotUsingShooter() {
  return frc2::cmd::Sequence(ledSub.SetBothToOff(),
                             shooterSub.GoToVelocityCmd([] { return 0_rpm; }));
}

frc2::CommandPtr RobotContainer::IntakeNote() {
  return frc2::cmd::Sequence(ledSub.SetBothToBlinkOrange(),
                             intakeSub.SuckInUntilNoteIsSeen(),
                             ledSub.SetBothToSolidOrange(), RumbleDriver());
}

frc2::CommandPtr RobotContainer::RumbleDriver() {
  return frc2::cmd::Sequence(
             frc2::cmd::RunOnce([this] {
               driverController.SetRumble(
                   frc::GenericHID::RumbleType::kBothRumble, 0.5);
             }),
             frc2::cmd::Wait(.5_s), frc2::cmd::RunOnce([this] {
               driverController.SetRumble(
                   frc::GenericHID::RumbleType::kBothRumble, 0.0);
             }))
      .FinallyDo([this] {
        driverController.SetRumble(frc::GenericHID::RumbleType::kBothRumble,
                                   0.0);
      });
}

frc2::CommandPtr RobotContainer::RumbleOperator() {
  return frc2::cmd::Sequence(
             frc2::cmd::RunOnce([this] {
               operatorController.SetRumble(
                   frc::GenericHID::RumbleType::kBothRumble, 0.5);
             }),
             frc2::cmd::Wait(.5_s), frc2::cmd::RunOnce([this] {
               operatorController.SetRumble(
                   frc::GenericHID::RumbleType::kBothRumble, 0.0);
             }))
      .FinallyDo([this] {
        operatorController.SetRumble(frc::GenericHID::RumbleType::kBothRumble,
                                     0.0);
      });
}

units::radian_t RobotContainer::ShouldFlipAngleForDriver(
    units::radian_t targetAngle) {
  auto ally = frc::DriverStation::GetAlliance();
  if (ally.has_value()) {
    if (ally.value() == frc::DriverStation::Alliance::kRed) {
      return targetAngle + 180_deg;
    }
  }
  return targetAngle;
}

DrivebaseSubsystem& RobotContainer::GetDrivebaseSubsystem() {
  return driveSub;
}

ShooterSubsystem& RobotContainer::GetShooterSubsystem() {
  return shooterSub;
}

DunkerSubsystem& RobotContainer::GetDunkerSubsystem() {
  return dunkSub;
}

Vision& RobotContainer::GetVisionSystem() {
  return vision;
}

str::DeadbandAndSquareFunc RobotContainer::DeadbandAndSquare(
    std::function<double()> joystickValue) {
  return [joystickValue]() {
    double deadband = frc::ApplyDeadband<double>(joystickValue(), 0.2);
    return std::abs(deadband) * deadband;
  };
}

std::function<frc2::CommandPtr()> RobotContainer::GetAStarCmd() {
  return [this] {
    return driveSub.PathfindToSafeSpot(
        [this] { return driveSub.CalculateClosestSafeSpot(); });
  };
}
