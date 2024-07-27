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
#include "subsystems/SubClimber.h"
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <frc/smartdashboard/SmartDashboard.h>

RobotContainer::RobotContainer(){

  pathplanner::NamedCommands::registerCommand("Intake", SubIntake::GetInstance().Intake());
  pathplanner::NamedCommands::registerCommand("IntakeSequence", cmd::CmdIntake());
  pathplanner::NamedCommands::registerCommand(
      "SetShooterToSpeakerSpeed",
      SubShooter::GetInstance().CmdSetShooterSpeaker().AndThen(
          frc2::cmd::Wait(1_s)));
  pathplanner::NamedCommands::registerCommand("FeedToShooter", SubFeeder::GetInstance().FeedToShooter().WithTimeout(0.2_s));
  pathplanner::NamedCommands::registerCommand("Shoot", cmd::CmdShootNeutral());
  pathplanner::NamedCommands::registerCommand("Feed", SubFeeder::GetInstance().FeedToShooter().WithTimeout(1_s));
  pathplanner::NamedCommands::registerCommand("FullSequenceShoot", cmd::CmdShootSpeakerAuto());
  pathplanner::NamedCommands::registerCommand(
      "SetSubwooferAngle",
      SubPivot::GetInstance().CmdSetPivotAngle(41_deg).AndThen(
          frc2::cmd::WaitUntil(
              [] { return SubPivot::GetInstance().IsOnTarget(); })));

  std::shared_ptr<pathplanner::PathPlannerPath> exampleChoreoTraj = pathplanner::PathPlannerPath::fromChoreoTrajectory("AA1.1");

  SubDrivebase::GetInstance().SetDefaultCommand(
      SubDrivebase::GetInstance().JoystickDrive(_driverController, false));
  
  ConfigureBindings();
  SubVision::GetInstance();

  _autoChooser.AddOption("M44Note", "M44Note");
  _autoChooser.AddOption("Dont Move", "Dont Move");
  _autoChooser.AddOption("S1 (C5 first)", "S1 (C5 first)");

  frc::SmartDashboard::PutData("Chosen Path", &_autoChooser);
}

void RobotContainer::ConfigureBindings() {
  //Driver

  //Triggers
  _driverController.RightTrigger().WhileTrue(cmd::CmdIntake());
  _driverController.LeftTrigger().WhileTrue(cmd::CmdOuttake());
  //Bumpers
   _driverController.LeftBumper().WhileTrue(cmd::CmdSourcePickUp());
  //Letters
  _driverController.Y().OnTrue(SubDrivebase::GetInstance().ResetGyroCmd());
  _driverController.A().OnTrue(SubPivot::GetInstance().CmdSetPivotAngle(70_deg));
  _driverController.B().OnTrue(SubPivot::GetInstance().CmdSetPivotAngle(10_deg));


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
  POVHelper::Up(&_operatorController).OnFalse(SubClimber::GetInstance().ClimberStop());
  // POVHelper::Up(&_operatorController).WhileTrue(SubClimber::GetInstance().ClimberManualDrive(-0.5));
  POVHelper::Up(&_operatorController).WhileTrue(cmd::CmdClimb());
  POVHelper::Down(&_operatorController).WhileTrue(SubClimber::GetInstance().ClimberManualDrive(0.5));
  POVHelper::Down(&_operatorController).OnFalse(SubClimber::GetInstance().ClimberStop());
  POVHelper::Right(&_operatorController).WhileTrue(SubClimber::GetInstance().ClimberAutoReset());
  // POVHelper::Down(&_operatorController).OnTrue(SubClimber::GetInstance().ClimberResetTop());

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

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  auto _autoSelected = _autoChooser.GetSelected();
  //units::second_t delay = _delayChooser.GetSelected() * 0.01_s;
  units::second_t delay = 0.00_s;
  return frc2::cmd::Wait(delay)
      .AndThen(pathplanner::PathPlannerAuto(_autoSelected).ToPtr())
      .AlongWith(SubClimber::GetInstance().ClimberAutoReset().AndThen(
          SubClimber::GetInstance().ClimberPosition(SubClimber::_ClimberPosStow)));
}

frc2::CommandPtr RobotContainer::Rumble(double force, units::second_t duration) {
return frc2::cmd::Run([this, force, duration]{  
    _driverController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, force);
    _operatorController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, force);}).WithTimeout(duration)
    .FinallyDo([this]{
    _driverController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0);
    _operatorController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0);});
}