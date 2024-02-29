// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>

#include "frc/smartdashboard/SmartDashboard.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/PrintCommand.h"
#include "subsystems/DrivebaseSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"

namespace autos {

enum CommandSelector {
  DO_NOTHING,
  SQUARE,
  CHOREO_TEST,
  AMP_SIDE,
  MIDDLE_OF_DRIVER_STATION
};

class Autos {
 public:
  explicit Autos(DrivebaseSubsystem& driveSub, ShooterSubsystem& shooterSub,
                 IntakeSubsystem& intakeSub)
      : m_driveSub(driveSub), m_shooterSub(shooterSub), m_intakeSub(intakeSub) {
    pathplanner::NamedCommands::registerCommand(
        "TestCommandPrint",
        frc2::PrintCommand("Test Print from PP Command").ToPtr());

    pathplanner::NamedCommands::registerCommand(
        "SpinUpShooter",
        shooterSub
            .GoToVelocityCmd([] { return constants::shooter::SHOOTER_SPEED; })
            .WithTimeout(2_s));

    pathplanner::NamedCommands::registerCommand(
        "IntakeNote", intakeSub.SuckInUntilNoteIsSeen());

    pathplanner::NamedCommands::registerCommand(
        "FeedNote", intakeSub.SuckInNotes().WithTimeout(1_s));

    GetSelectedAutoCmd = frc2::cmd::Select<CommandSelector>(
        [this] { return chooser.GetSelected(); },
        std::pair{DO_NOTHING,
                  frc2::cmd::Print("ERROR: DO NOTHING AUTO SELECTED! YOU "
                                   "PROBABLY DIDNT MEAN THIS")},
        std::pair{SQUARE, pathplanner::PathPlannerAuto{"Square"}.ToPtr()},
        std::pair{CHOREO_TEST, m_driveSub.FollowChoreoTrajectory(
                                   [] { return "ChoreoTestPath"; })},
        std::pair{AMP_SIDE, pathplanner::PathPlannerAuto{"AmpSide"}.ToPtr()},
        std::pair{
            MIDDLE_OF_DRIVER_STATION,
            pathplanner::PathPlannerAuto{"MiddleOfDriverStation"}.ToPtr()});

    chooser.SetDefaultOption("Do Nothing", CommandSelector::DO_NOTHING);
    chooser.AddOption("Drive in Square", CommandSelector::SQUARE);
    chooser.AddOption("Choreo Test", CommandSelector::CHOREO_TEST);
    chooser.AddOption("Amp Side", CommandSelector::AMP_SIDE);
    chooser.AddOption("Middle of Driver Station",
                      CommandSelector::MIDDLE_OF_DRIVER_STATION);

    frc::SmartDashboard::PutData("Auto Chooser", &chooser);
  }

  DrivebaseSubsystem& m_driveSub;
  ShooterSubsystem& m_shooterSub;
  IntakeSubsystem& m_intakeSub;

  frc::SendableChooser<CommandSelector> chooser;

  frc2::CommandPtr GetSelectedAutoCmd{frc2::cmd::None()};
};
}  // namespace autos
