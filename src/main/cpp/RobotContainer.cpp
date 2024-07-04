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
  pathplanner::NamedCommands::registerCommand("FeedToShooter", SubFeeder::GetInstance().FeedToShooter().WithTimeout(0.2_s));
  pathplanner::NamedCommands::registerCommand("Shoot", cmd::CmdShootNeutral());
  pathplanner::NamedCommands::registerCommand("FullSequenceShoot", cmd::CmdShootSpeakerAuto());
  pathplanner::NamedCommands::registerCommand("SetSubwooferAngle", SubShooter::GetInstance().CmdSetShooterOff());

  std::shared_ptr<pathplanner::PathPlannerPath> exampleChoreoTraj = pathplanner::PathPlannerPath::fromChoreoTrajectory("AA1.1");

  SubDrivebase::GetInstance().SetDefaultCommand(
      SubDrivebase::GetInstance().JoystickDrive(_driverController, false));
  
  ConfigureBindings();
  SubVision::GetInstance();

  _autoChooser.AddOption("AA1", "3CloseNote");
  _autoChooser.AddOption("1Close2Far", "1Close2Far");
  _autoChooser.AddOption("M44Note", "M44Note");
  _autoChooser.AddOption("Example Path", "Example Path");

  frc::SmartDashboard::PutData("Chosen Path", &_autoChooser);
}

void RobotContainer::ConfigureBindings() {
  //Driver

  //Triggers
  _driverController.RightTrigger().WhileTrue(cmd::CmdIntake());
  //Bumpers
  
  //Letters
  _driverController.Y().OnTrue(SubDrivebase::GetInstance().ResetGyroCmd());
  _driverController.B().WhileTrue(SubPivot::GetInstance().CmdSetPivotAngle(40_deg)); //Remove later
  _driverController.A().WhileTrue(SubPivot::GetInstance().CmdSetPivotAngle(20_deg)); //Remove later
  //POV

  //Operator

  //Triggers
  _operatorController.RightTrigger().WhileTrue(cmd::CmdShootSpeaker(_driverController));
  
  //Bumpers
  _operatorController.RightBumper().WhileTrue(cmd::CmdShootPassing());
  _operatorController.LeftBumper().WhileTrue(cmd::CmdShootNeutral());

  //Letters
  _operatorController.A().WhileTrue(cmd::CmdShootSubwoofer());
  _operatorController.B().WhileTrue(cmd::CmdShootAmp());
  //POV
  POVHelper::Left(&_operatorController).OnTrue(SubShooter::GetInstance().CmdSetShooterOff());
  POVHelper::Right(&_operatorController).WhileTrue(cmd::CmdOuttake());
  // _operatorController.A().WhileTrue(SubPivot::GetInstance().CmdSetPivotAngle(65_deg));
  // _operatorController.B().WhileTrue(cmd::CmdShootAmp());
  _operatorController.Y().WhileTrue(SubClimber::GetInstance().ClimberManualDrive(0.5));
  _operatorController.Y().OnFalse(SubClimber::GetInstance().ClimberStop());
  _operatorController.X().OnFalse(SubClimber::GetInstance().ClimberStop());
  _operatorController.X().WhileTrue(SubClimber::GetInstance().ClimberManualDrive(-0.5));
  //POV
  POVHelper::Left(&_operatorController).OnTrue(SubShooter::GetInstance().CmdSetShooterOff());
  POVHelper::Right(&_operatorController).WhileTrue(cmd::CmdOuttake());
  POVHelper::Down(&_operatorController).WhileTrue(SubClimber::GetInstance().ClimberAutoReset());
  POVHelper::Up(&_operatorController).OnTrue(SubClimber::GetInstance().ClimberResetZero());
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
  units::second_t delay = 0.01_s;
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