// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "subsystems/SubIntake.h"
#include <frc2/command/Commands.h>
#include "subsystems/SubShooter.h"
#include "subsystems/SubPivot.h"
#include "commands/ShooterCommands.h"
#include "utilities/POVHelper.h"
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubFeeder.h"
#include "subsystems/SubVision.h"
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <frc/smartdashboard/SmartDashboard.h>

RobotContainer::RobotContainer(){
  SubDrivebase::GetInstance().SetDefaultCommand(
      SubDrivebase::GetInstance().JoystickDrive(_driverController, false));

  ConfigureBindings();
  SubVision::GetInstance();

  _autoChooser.AddOption("Pathplanner_auto_test", "Pathplanner_auto_test");
  _autoChooser.AddOption("New Auto", "New Auto");
  frc::SmartDashboard::PutData("Chosen Path", &_autoChooser);
}

void RobotContainer::ConfigureBindings() {
  //Driver

  //Triggers
  _driverController.RightTrigger().WhileTrue(cmd::CmdIntake());
  //Bumpers
  
  //Letters
  
  //POV

  //Operator

  //Triggers
  _operatorController.RightTrigger().WhileTrue(cmd::CmdShootSpeaker());
  //Bumpers
  _operatorController.RightBumper().WhileTrue(cmd::CmdShootPassing());
  _operatorController.LeftBumper().WhileTrue(cmd::CmdShootNeutral());
  //Letters
  _operatorController.A().WhileTrue(SubPivot::GetInstance().CmdSetPivotAngle(65_deg));
  _operatorController.B().WhileTrue(cmd::CmdShootAmp());
  //POV
  POVHelper::Left(&_operatorController).OnTrue(SubShooter::GetInstance().CmdSetShooterOff());
  POVHelper::Right(&_operatorController).WhileTrue(cmd::CmdOuttake());

  //Robot triggers
  frc2::Trigger{[]{return SubFeeder::GetInstance().CheckHasNote();}}.OnTrue(Rumble(1, 0.3_s));  
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  auto _autoSelected = _autoChooser.GetSelected();
  //units::second_t delay = _delayChooser.GetSelected() * 0.01_s;
  units::second_t delay = 0.01_s;
  return frc2::cmd::Wait(delay)
      .AndThen(pathplanner::PathPlannerAuto(_autoSelected).ToPtr());
      // .AlongWith(SubClimber::GetInstance().ClimberAutoReset().AndThen(
      //     SubClimber::GetInstance().ClimberPosition(SubClimber::_ClimberPosStow)));
}

frc2::CommandPtr RobotContainer::Rumble(double force, units::second_t duration) {
return frc2::cmd::Run([this, force, duration]{  
    _driverController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, force);
    _operatorController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, force);}).WithTimeout(duration)
    .FinallyDo([this]{
    _driverController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0);
    _operatorController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0);});
}