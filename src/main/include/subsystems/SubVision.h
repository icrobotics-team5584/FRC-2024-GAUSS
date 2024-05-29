// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <photon/PhotonCamera.h>
#include <units/length.h>
#include <units/angle.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <map>
#include <photon/PhotonPoseEstimator.h>
#include <photon/simulation/SimPhotonCamera.h>
#include <photon/simulation/SimVisionSystem.h>
#include <photon/simulation/SimVisionTarget.h>
#include <frc/Filesystem.h>
#include <frc2/command/SubsystemBase.h>


class SubVision : public frc2::SubsystemBase {
 public:
  SubVision();


  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SimulationPeriodic() override;

private:
  std::string CAM_NAME1 = "arducam";

  frc::Transform3d _camToBot{{-196_mm, 41_mm, -680_mm}, {}}; // arducam
  
  frc::AprilTagFieldLayout _tagLayout{frc::filesystem::GetDeployDirectory() + "/AprilTags.json"};

  // photonlib::PhotonPoseEstimator _visionPoseEstimator{
  //     _tagLayout,
  //     photonlib::PoseStrategy::MULTI_TAG_PNP,
  //     photonlib::PhotonCamera{CAM_NAME1},
  //     _camToBot.Inverse()};

  photon::SimVisionSystem _visionSim{CAM_NAME1, 45_deg, _camToBot, 15_m,
                                        360,         240,    0.0001};
                                        
};
