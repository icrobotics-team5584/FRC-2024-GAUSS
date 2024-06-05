// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "subsystems/SubIntake.h"
#include <frc2/command/Commands.h>
#include "subsystems/SubShooter.h"
#include "subsystems/SubPivot.h"
#include "ShooterCommands.h"
#include "utilities/POVHelper.h"
#include "subsystems/SubDrivebase.h"

RobotContainer::RobotContainer() {

    SubDrivebase::GetInstance();
  SubDrivebase::GetInstance().SetDefaultCommand(
    SubDrivebase::GetInstance().JoystickDrive(_driverController, false)
  );

  
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  //Triggers
  _driverController.RightTrigger().WhileTrue(cmd::CmdShootSpeaker());
  _driverController.RightTrigger().WhileTrue(cmd::CmdIntake());
  _driverController.LeftTrigger().WhileTrue(SubIntake::GetInstance().Intake().AndThen(Rumble(1, 0.3_s)));
  //Bumpers
  _driverController.RightBumper().WhileTrue(cmd::CmdShootPassing());
  _driverController.LeftBumper().WhileTrue(cmd::CmdShootNeutral());
  //Letters
  _driverController.A().WhileTrue(SubPivot::GetInstance().CmdSetPivotAngle(65_deg));
  _driverController.B().WhileTrue(cmd::CmdShootAmp());
  //POV
  POVHelper::Left(&_driverController).ToggleOnTrue(SubShooter::GetInstance().CmdSetShooterOff());
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