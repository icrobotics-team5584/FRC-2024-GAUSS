// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubVision.h"
#include "subsystems/SubDrivebase.h"

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
void SubVision::Periodic() {}



void SubVision::SimulationPeriodic() {
  _visionSim.ProcessFrame(SubDrivebase::GetInstance().GetPose());
};


