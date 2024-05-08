// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubIntake.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <rev/CANSparkLowLevel.h>


using namespace frc2::cmd;

SubIntake::SubIntake() {
  frc::SmartDashboard::PutString("Intake/Intake Deploy State: ", "Intake retracted");
  _intakeMotorSpin.RestoreFactoryDefaults();
  _intakeMotorSpin.SetSmartCurrentLimit(60);
  _intakeMotorSpin.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus0, 500);
  _intakeMotorSpin.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus1, 500);
  _intakeMotorSpin.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus2, 500);
  _intakeMotorSpin.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus3, 500);
  _intakeMotorSpin.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus4, 500);
  _intakeMotorSpin.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus5, 500);
  _intakeMotorSpin.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus6, 500);
}




// This method will be called once per scheduler run
void SubIntake::Periodic() {}
