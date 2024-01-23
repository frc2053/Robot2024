// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/RobotBase.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/estimation/VisionEstimation.h>
#include <photon/simulation/VisionSystemSim.h>
#include <photon/simulation/VisionTargetSim.h>
#include <photon/targeting/PhotonPipelineResult.h>

#include <limits>
#include <memory>

#include "Constants.h"

class Vision {
 public:
  Vision() {
    frc::AprilTagFieldLayout layout =
        frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo);

    photonEstimator = std::make_unique<photon::PhotonPoseEstimator>(
        layout, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
        photon::PhotonCamera(constants::vision::kCameraName),
        constants::vision::kRobotToCam);
    camera = photonEstimator->GetCamera();

    photonEstimator->SetMultiTagFallbackStrategy(
        photon::PoseStrategy::LOWEST_AMBIGUITY);

    if (frc::RobotBase::IsSimulation()) {
      visionSim = std::make_unique<photon::VisionSystemSim>("main");

      visionSim->AddAprilTags(layout);

      cameraProp = std::make_unique<photon::SimCameraProperties>();

      cameraProp->SetCalibration(960, 720, frc::Rotation2d{90_deg});
      cameraProp->SetCalibError(.35, .10);
      cameraProp->SetFPS(45_Hz);
      cameraProp->SetAvgLatency(20_ms);
      cameraProp->SetLatencyStdDev(15_ms);

      cameraSim = std::make_shared<photon::PhotonCameraSim>(camera.get(),
                                                            *cameraProp.get());

      visionSim->AddCamera(cameraSim.get(), robotToCam);
      cameraSim->EnableDrawWireframe(true);
    }
  }

  photon::PhotonPipelineResult GetLatestResult() {
    return camera->GetLatestResult();
  }

  std::optional<photon::EstimatedRobotPose> GetEstimatedGlobalPose() {
    auto visionEst = photonEstimator->Update();
    units::second_t latestTimestamp = camera->GetLatestResult().GetTimestamp();
    bool newResult =
        units::math::abs(latestTimestamp - lastEstTimestamp) > 1e-5_s;
    if (frc::RobotBase::IsSimulation()) {
      if (visionEst.has_value()) {
        GetSimDebugField()
            .GetObject("VisionEstimation")
            ->SetPose(visionEst.value().estimatedPose.ToPose2d());
      } else {
        if (newResult) {
          GetSimDebugField().GetObject("VisionEstimation")->SetPoses({});
        }
      }
    }
    if (newResult) {
      lastEstTimestamp = latestTimestamp;
    }
    return visionEst;
  }

  Eigen::Matrix<double, 3, 1> GetEstimationStdDevs(frc::Pose2d estimatedPose) {
    Eigen::Matrix<double, 3, 1> estStdDevs =
        constants::vision::kSingleTagStdDevs;
    auto targets = GetLatestResult().GetTargets();
    int numTags = 0;
    units::meter_t avgDist = 0_m;
    for (const auto& tgt : targets) {
      auto tagPose =
          photonEstimator->GetFieldLayout().GetTagPose(tgt.GetFiducialId());
      if (tagPose.has_value()) {
        numTags++;
        avgDist += tagPose.value().ToPose2d().Translation().Distance(
            estimatedPose.Translation());
      }
    }
    if (numTags == 0) {
      return estStdDevs;
    }
    avgDist /= numTags;
    if (numTags > 1) {
      estStdDevs = constants::vision::kMultiTagStdDevs;
    }
    if (numTags == 1 && avgDist > 4_m) {
      estStdDevs = (Eigen::MatrixXd(3, 1) << std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max())
                       .finished();
    } else {
      estStdDevs = estStdDevs * (1 + (avgDist.value() * avgDist.value() / 30));
    }
    return estStdDevs;
  }

  void SimPeriodic(frc::Pose2d robotSimPose) {
    visionSim->Update(robotSimPose);
  }

  void ResetSimPose(frc::Pose2d pose) {
    if (frc::RobotBase::IsSimulation()) {
      visionSim->ResetRobotPose(pose);
    }
  }

  frc::Field2d& GetSimDebugField() { return visionSim->GetDebugField(); }

 private:
  frc::Transform3d robotToCam = constants::vision::kRobotToCam;
  std::unique_ptr<photon::PhotonPoseEstimator> photonEstimator;
  std::shared_ptr<photon::PhotonCamera> camera;
  std::unique_ptr<photon::VisionSystemSim> visionSim;
  std::unique_ptr<photon::SimCameraProperties> cameraProp;
  std::shared_ptr<photon::PhotonCameraSim> cameraSim;
  units::second_t lastEstTimestamp{0_s};
};
