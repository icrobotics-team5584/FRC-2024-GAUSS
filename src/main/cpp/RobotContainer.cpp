// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "subsystems/SubIntake.h"
#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  _driverController.LeftTrigger().WhileTrue(SubIntake::GetInstance().Intake().AndThen(Rumble(1, 0.3_s)));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}

frc2::CommandPtr RobotContainer::Rumble(double force, units::second_t duration) {
return frc2::cmd::Run([this, force, duration]{  
    //_driverController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, force);
    _operatorController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, force);}).WithTimeout(duration)
    .FinallyDo([this]{
    //_driverController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0);
    _operatorController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0);});
}