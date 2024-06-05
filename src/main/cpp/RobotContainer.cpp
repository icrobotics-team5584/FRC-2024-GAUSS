// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "subsystems/SubIntake.h"
#include <frc2/command/Commands.h>
#include "subsystems/SubClimber.h"
#include "subsystems/SubShooter.h"
#include "subsystems/SubPivot.h"
#include "subsystems/SubDrivebase.h"

RobotContainer::RobotContainer() {

    SubDrivebase::GetInstance();
  SubDrivebase::GetInstance().SetDefaultCommand(
    SubDrivebase::GetInstance().JoystickDrive(_driverController, false)
  );

  SubClimber::GetInstance();
  
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // _driverController.LeftBumper().WhileTrue(SubShooter::GetInstance().CmdSetShooterSpeaker());
  // _driverController.RightBumper().WhileTrue(SubShooter::GetInstance().CmdSetShooterAmp());
  // _driverController.LeftTrigger().WhileTrue(SubShooter::GetInstance().CmdSetShooterOff());
  // _driverController.RightTrigger().WhileTrue(SubShooter::GetInstance().CmdSetShooterPassing());
  // _driverController.Y().WhileTrue(SubPivot::GetInstance().CmdSetPivotAngle(10_deg));
  // _driverController.A().WhileTrue(SubPivot::GetInstance().CmdSetPivotAngle(40_deg));
  // _driverController.LeftTrigger().WhileTrue(SubIntake::GetInstance().Intake().AndThen(Rumble(1, 0.3_s)));

  _driverController.A().OnTrue(SubClimber::GetInstance().ClimberPosition(1.5_m));
  _driverController.B().OnTrue(SubClimber::GetInstance().ClimberPosition(0_m));
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