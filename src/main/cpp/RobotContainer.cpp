// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "subsystems/SubIntake.h"
#include <frc2/command/Commands.h>
#include "subsystems/SubShooter.h"
#include "subsystems/SubPivot.h"
#include "commands/ShooterCommands.h"
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubFeeder.h"
#include "subsystems/SubVision.h"
#include "subsystems/SubClimber.h"
#include <pathplanner/lib/auto/NamedCommands.h>
#include <frc/smartdashboard/SmartDashboard.h>

RobotContainer::RobotContainer(){

  // Initialize subsystems
  SubVision::GetInstance();
  SubDrivebase::GetInstance().SetDefaultCommand(
      SubDrivebase::GetInstance().JoystickDrive(_driverController));

  // Auto chooser
  _autoChooser.AddOption("bigPath", "bigPath");
  _autoChooser.SetDefaultOption("indivPaths", "indivPaths");
  frc::SmartDashboard::PutData("Chosen Path", &_autoChooser);

  // Register pathplanner named commands
  using ppcmd = pathplanner::NamedCommands;
  ppcmd::registerCommand("intake", cmd::CmdIntake());
  ppcmd::registerCommand("feedOnceOnTarget", cmd::CmdFeedOnceOnTarget());
  ppcmd::registerCommand("setShooterSpeaker", SubShooter::GetInstance().CmdSetShooterSpeaker());
  ppcmd::registerCommand("FullSequenceShoot", cmd::CmdShootSpeakerAuto());

  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  
  //Driver

  //Triggers
  _driverController.RightTrigger().WhileTrue(cmd::CmdIntake());
  _driverController.LeftTrigger().WhileTrue(cmd::CmdOuttake());
  //Bumpers
   _driverController.LeftBumper().WhileTrue(cmd::CmdSourcePickUp());
  //Letters
  _driverController.A().OnTrue(SubPivot::GetInstance().CmdSetPivotAngle(70_deg));
  _driverController.B().OnTrue(SubPivot::GetInstance().CmdSetPivotAngle(10_deg));
  _driverController.Y().OnTrue(SubClimber::GetInstance().ClimberPosition(SubClimber::TOP_HEIGHT));
  _driverController.X().OnTrue(SubClimber::GetInstance().ClimberPosition(SubClimber::BASE_HEIGHT));


  //POV

  //Operator

  //Triggers
  _operatorController.LeftTrigger().WhileTrue(cmd::CmdShootNeutral());
  _operatorController.RightTrigger().WhileTrue(cmd::CmdShootSpeaker(_driverController));
  
  //Bumpers
  _operatorController.LeftBumper().WhileTrue(cmd::CmdShootPassing());
  _operatorController.RightBumper().WhileTrue(cmd::CmdShootSubwoofer());

  //Letters
  _operatorController.A().WhileTrue(cmd::CmdShootAmp());
  _operatorController.B().WhileTrue(cmd::CmdOuttake());
  _operatorController.Y().OnTrue(SubShooter::GetInstance().CmdSetShooterOff());

  //POV
  // _operatorController.A().WhileTrue(SubPivot::GetInstance().CmdSetPivotAngle(65_deg));
  // _operatorController.B().WhileTrue(cmd::CmdShootAmp());
  
  

  //POV
  _operatorController.POVUp().OnFalse(SubClimber::GetInstance().ClimberStop());
  _operatorController.POVUp().WhileTrue(cmd::CmdClimb());
  _operatorController.POVDown().WhileTrue(SubClimber::GetInstance().ClimberManualDrive(0.5));
  _operatorController.POVDown().OnFalse(SubClimber::GetInstance().ClimberStop());
  _operatorController.POVRight().WhileTrue(SubClimber::GetInstance().ClimberAutoReset());

  //Triggers
  frc2::Trigger{[]{return SubFeeder::GetInstance().CheckHasNote();}}.OnTrue(Rumble(1, 0.3_s));  

  frc2::Trigger(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop(), [=, this] {
    return (_operatorController.GetLeftY() < -0.2 || _operatorController.GetLeftY() > 0.2) &&
    !(_operatorController.GetRightY() < -0.2 || _operatorController.GetRightY() > 0.2);
  }).WhileTrue(SubClimber::GetInstance().ClimberJoystickDriveLeft(_operatorController));


  frc2::Trigger(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop(), [=, this] {
    return (_operatorController.GetRightY() < -0.2 || _operatorController.GetRightY() > 0.2) &&
    !(_operatorController.GetLeftY() < -0.2 || _operatorController.GetLeftY() > 0.2);
  }).WhileTrue(SubClimber::GetInstance().ClimberJoystickDriveRight(_operatorController));

  frc2::Trigger(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop(), [=, this] {
    return (_operatorController.GetRightY() < -0.2 || _operatorController.GetRightY() > 0.2) &&
           (_operatorController.GetLeftY() < -0.2 || _operatorController.GetLeftY() > 0.2);
  }).WhileTrue(SubClimber::GetInstance().ClimberJoystickDrive(_operatorController));
}

pathplanner::PathPlannerAuto RobotContainer::GetAutonomousCommand() {
  auto _autoSelected = _autoChooser.GetSelected();
  auto followPath = pathplanner::PathPlannerAuto(_autoSelected);

  followPath.event("beginVisionAim").OnTrue(frc2::cmd::RunOnce([] {
    SubDrivebase::GetInstance().SetPathplannerRotationFeedbackSource([] {
      return SubDrivebase::GetInstance().CalcRotateSpeed(
          SubVision::GetInstance().GetSpeakerYaw().value_or(0_deg));
    });
  }));

  followPath.event("endVisionAim").OnTrue(frc2::cmd::RunOnce([] {
    SubDrivebase::GetInstance().ResetPathplannerRotationFeedbackSource();
  }));

  followPath.isRunning().OnTrue(SubClimber::GetInstance().ClimberAutoReset().AndThen(
      SubClimber::GetInstance().ClimberPosition(SubClimber::STOW_HEIGHT)));

  return std::move(followPath);
}

frc2::CommandPtr RobotContainer::Rumble(double force, units::second_t duration) {
return frc2::cmd::Run([this, force, duration]{  
    _driverController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, force);
    _operatorController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, force);}).WithTimeout(duration)
    .FinallyDo([this]{
    _driverController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0);
    _operatorController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0);});
}