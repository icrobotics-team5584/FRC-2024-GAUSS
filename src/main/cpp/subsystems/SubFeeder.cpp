// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubFeeder.h"
#include <frc2/command/commands.h>
#include <frc/smartdashboard/SmartDashboard.h>



SubFeeder::SubFeeder() {
    frc::SmartDashboard::PutData("Feeder/Motor", (wpi::Sendable*)&_feederMotor);
    _feederMotor.SetInverted(true);
}

frc2::CommandPtr SubFeeder::FeedToIntake() {
return Run([this]{ _feederMotor.Set(0.7);}).FinallyDo([this]{_feederMotor.Set(0);});
}

frc2::CommandPtr SubFeeder::FeedToShooter() {
return Run([this]{ _feederMotor.Set(-0.7);}).FinallyDo([this]{_feederMotor.Set(0);});
}

// This method will be called once per scheduler run
void SubFeeder::Periodic() {
}

bool SubFeeder::CheckHasNote(){
    if(_feederPointSwitch.Get()){
    return true;
  }

  return false;
}
