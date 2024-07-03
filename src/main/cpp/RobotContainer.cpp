// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "subsystems/SubIntake.h"
#include <frc2/command/Commands.h>
#include "subsystems/SubClimber.h"
#include "subsystems/SubShooter.h"
#include "subsystems/SubPivot.h"
#include "commands/ShooterCommands.h"
#include "utilities/POVHelper.h"
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubFeeder.h"
#include "subsystems/SubVision.h"

RobotContainer::RobotContainer(){
  SubDrivebase::GetInstance().SetDefaultCommand(
      SubDrivebase::GetInstance().JoystickDrive(_driverController, false));

  SubClimber::GetInstance();
  
  ConfigureBindings();
  SubVision::GetInstance();
}

void RobotContainer::ConfigureBindings() {
  //Driver

  //Triggers
  _driverController.RightTrigger().WhileTrue(cmd::CmdIntake());
  _driverController.LeftTrigger().WhileTrue(cmd::CmdAimAtSpeakerWithVision(_driverController));
  //Bumpers
  
  //Letters
  _driverController.Y().OnTrue(SubDrivebase::GetInstance().ResetGyroCmd());
  //POV

  //Operator

  //Triggers
  _operatorController.RightTrigger().WhileTrue(cmd::CmdShootSpeaker(_driverController));
  
  
  //Bumpers
  _operatorController.RightBumper().WhileTrue(cmd::CmdShootPassing());
  _operatorController.LeftBumper().WhileTrue(cmd::CmdShootNeutral());
  //Letters
  // _operatorController.A().WhileTrue(SubPivot::GetInstance().CmdSetPivotAngle(65_deg));
  // _operatorController.B().WhileTrue(cmd::CmdShootAmp());
  _operatorController.A().WhileTrue(SubClimber::GetInstance().ClimberManualDrive(1));
  _operatorController.A().OnFalse(SubClimber::GetInstance().ClimberStop());
  _operatorController.B().OnFalse(SubClimber::GetInstance().ClimberStop());
  _operatorController.B().WhileTrue(SubClimber::GetInstance().ClimberManualDrive(-1));
  _operatorController.Y().OnTrue(SubClimber::GetInstance().ClimberPosition(0.4_m));
  _operatorController.Y().OnTrue(SubClimber::GetInstance().(0.4_m));
  _operatorController.X().OnTrue(SubClimber::GetInstance().ClimberPosition(0.05_m));
  //POV
  POVHelper::Left(&_operatorController).OnTrue(SubShooter::GetInstance().CmdSetShooterOff());
  POVHelper::Right(&_operatorController).WhileTrue(cmd::CmdOuttake());
  POVHelper::Down(&_operatorController).WhileTrue(SubClimber::GetInstance().ClimberAutoReset());
  POVHelper::Up(&_operatorController).OnTrue(SubClimber::GetInstance().ClimberResetZero());

  //Robot triggers
  frc2::Trigger{[]{return SubFeeder::GetInstance().CheckHasNote();}}.OnTrue(Rumble(1, 0.3_s));  
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured"); 
}

frc2::CommandPtr RobotContainer::Rumble(double force, units::second_t duration) {
return frc2::cmd::Run([this, force, duration]{  
    _driverController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, force);
    _operatorController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, force);}).WithTimeout(duration)
    .FinallyDo([this]{
    _driverController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0);
    _operatorController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0);});
}