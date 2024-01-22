// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <str/DataUtils.h>

void Robot::RobotInit() {
  str::DataUtils::SetupDataLogging();
  str::DataUtils::LogGitInfo();
  AddPeriodic([this] { m_container.GetDrivebaseSubsystem().UpdateOdometry(); },
              1 / 250_Hz);
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
  nt::NetworkTableInstance::GetDefault().Flush();

  auto visionEst = m_container.GetVisionSystem().GetEstimatedGlobalPose();
  if (visionEst.has_value()) {
    auto est = visionEst.value();
    auto estPose = est.estimatedPose.ToPose2d();
    auto estStdDevs =
        m_container.GetVisionSystem().GetEstimationStdDevs(estPose);
    m_container.GetDrivebaseSubsystem().AddVisionMeasurement(
        est.estimatedPose.ToPose2d(), est.timestamp, estStdDevs);
  }
}

void Robot::SimulationPeriodic() {
  m_container.GetVisionSystem().SimPeriodic(
      m_container.GetDrivebaseSubsystem().GetRobotPose());
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
