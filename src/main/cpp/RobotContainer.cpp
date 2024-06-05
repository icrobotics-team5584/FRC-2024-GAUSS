// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include "subsystems/SubShooter.h"
#include "subsystems/SubPivot.h"
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubVision.h"

RobotContainer::RobotContainer() {
  ConfigureBindings();
  SubVision::GetInstance();
}

void RobotContainer::ConfigureBindings() {
  _driverController.LeftBumper().WhileTrue(SubShooter::GetInstance().CmdSetShooterSpeaker());
  _driverController.RightBumper().WhileTrue(SubShooter::GetInstance().CmdSetShooterAmp());
  _driverController.LeftTrigger().WhileTrue(SubShooter::GetInstance().CmdSetShooterOff());
  _driverController.RightTrigger().WhileTrue(SubShooter::GetInstance().CmdSetShooterPassing());
  _driverController.Y().WhileTrue(SubPivot::GetInstance().CmdSetPivotAngle(10_deg));
  _driverController.A().WhileTrue(SubPivot::GetInstance().CmdSetPivotAngle(40_deg));
  SubDrivebase::GetInstance();
  SubDrivebase::GetInstance().SetDefaultCommand(
    SubDrivebase::GetInstance().JoystickDrive(_driverController, false)
  );
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
