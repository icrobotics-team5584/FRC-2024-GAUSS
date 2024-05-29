// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include "subsystems/SubShooter.h"
#include "subsystems/SubPivot.h"
#include "subsystems/SubDrivebase.h"

RobotContainer::RobotContainer() {
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  _controller.LeftBumper().WhileTrue(SubShooter::GetInstance().CmdSetShooterSpeaker());
  _controller.RightBumper().WhileTrue(SubShooter::GetInstance().CmdSetShooterAmp());
  _controller.LeftTrigger().WhileTrue(SubShooter::GetInstance().CmdSetShooterOff());
  _controller.RightTrigger().WhileTrue(SubShooter::GetInstance().CmdSetShooterPassing());
  _controller.Y().WhileTrue(SubPivot::GetInstance().CmdSetPivotAngle(10_deg));
  _controller.A().WhileTrue(SubPivot::GetInstance().CmdSetPivotAngle(40_deg));
  SubDrivebase::GetInstance();
  SubDrivebase::GetInstance().SetDefaultCommand(
    SubDrivebase::GetInstance().JoystickDrive(driverController, false)
  );
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
