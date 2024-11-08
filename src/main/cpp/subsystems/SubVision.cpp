// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubVision.h"
#include "subsystems/SubDrivebase.h"
#include <photon/simulation/VisionTargetSim.h>
#include <frc/DriverStation.h>

SubVision::SubVision() {
  // _visionSim.AddAprilTags(_tagLayout);
  // _visionSim.AddCamera(&_cameraSim, _camToBot.Inverse());
  
  // for (auto target : _visionSim.GetVisionTargets()) {
  //   SubDrivebase::GetInstance().DisplayPose(fmt::format("tag{}", target.fiducialId),
  //                                           target.GetPose().ToPose2d());
  // }
}

// This method will be called once per scheduler run
void SubVision::Periodic() {
  // _latestResults = _camera.GetAllUnreadResults();
  frc::SmartDashboard::PutNumber("Vision/Speaker Pitch", GetSpeakerPitch().value_or(-1000_deg).value());
  frc::SmartDashboard::PutNumber("Target/YawOnTarget", IsFacingTarget());
}

void SubVision::SimulationPeriodic() {
  // _visionSim.Update(SubDrivebase::GetInstance().GetPose());
};

// get latest result
std::optional<photon::PhotonPipelineResult> SubVision::GetLatestResult() {
  if (!_latestResults.empty()) {
    return _latestResults.back();
  }
  return std::nullopt;
}

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
  auto latestCameraResultOpt = GetLatestResult();
  if (!latestCameraResultOpt.has_value()) {
    return std::nullopt;
  }
  auto latestCameraResult = latestCameraResultOpt.value();
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

std::optional<units::degree_t> SubVision::GetLatestSpeakerPitch(){
  static frc::Timer timer;
  static std::optional<units::degree_t> latestPitch = std::nullopt;
  auto optionalSpeakerPitch = GetSpeakerPitch();
  if (optionalSpeakerPitch.has_value()) {
      latestPitch = optionalSpeakerPitch.value();
      timer.Restart();
  }
  if (timer.Get() >= 10_s) {
      latestPitch = std::nullopt;
  }
  return latestPitch;
}

bool SubVision::IsFacingTarget(){
  if (-2_deg < GetSpeakerYaw() && GetSpeakerYaw() < 2_deg){
    return true;
  } else {
    return false;
  }
}