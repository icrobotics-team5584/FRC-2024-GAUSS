// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubVision.h"
#include "subsystems/SubDrivebase.h"
#include <frc/DriverStation.h>

SubVision::SubVision() {
  for (int i = 0; i <= 18; i++) {
    auto pose = _tagLayout.GetTagPose(i);
    if (pose.has_value()) {
      photon::SimVisionTarget simTag{pose.value(), 8_in, 8_in, i};
      _visionSim.AddSimVisionTarget(simTag);
      SubDrivebase::GetInstance().DisplayPose(fmt::format("tag{}", i), pose.value().ToPose2d());
    }
  }
}

// This method will be called once per scheduler run
void SubVision::Periodic() {
  const auto& result = camera.GetLatestResult();
  double _pitchtotarget = {result.GetBestTarget().GetPitch()};
  frc::SmartDashboard::PutNumber("Vision/April Pitch", _pitchtotarget);
  frc::SmartDashboard::PutNumber("Vision/Speaker Pitch", GetSpeakerPitch().value_or(-1000_deg).value());
  }

void SubVision::SimulationPeriodic() {
  _visionSim.ProcessFrame(SubDrivebase::GetInstance().GetPose());
};

std::optional<photon::PhotonTrackedTarget> SubVision::GetSpeakerTarget() {
  auto alliance = frc::DriverStation::GetAlliance();
  std::array <int, 2> desiredIDs{-1, -1};
  if(alliance){
    if(alliance.value() == frc::DriverStation::Alliance::kBlue) {
      desiredIDs[0] = 7;
    }
    else if(alliance.value() == frc::DriverStation::Alliance::kRed){
      desiredIDs[0] = 4;
    }
  }
  else { //This is a case where the alliance is unknown
      desiredIDs[0] = 4;
      desiredIDs[1] = 7;
    }
  auto latestCameraResult = camera.GetLatestResult();
  auto latestTargets = latestCameraResult.GetTargets();
  auto checkRightApriltag = [desiredIDs](photon::PhotonTrackedTarget tag){
    return (tag.GetFiducialId() == desiredIDs[0] || tag.GetFiducialId() == desiredIDs[1]);
  };
  auto tagResult = std::ranges::find_if(latestTargets, checkRightApriltag);

  if (tagResult != latestTargets.end()) {
    return *tagResult;
  }
  else {
    return {};
  }

}

std::optional<units::degree_t> SubVision::GetSpeakerYaw(){
  auto tagResult = SubVision::GetInstance().GetSpeakerTarget();
  if (tagResult){
    return tagResult.value().GetYaw() * 1_deg;
  } else {
    return {};
  }
}

std::optional<units::degree_t> SubVision::GetSpeakerPitch(){
  auto tagResult = SubVision::GetInstance().GetSpeakerTarget();
  if (tagResult){
    return tagResult.value().GetPitch() * 1_deg;
  } else {
    return {};
  }
}