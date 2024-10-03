// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubIntake.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <rev/CANSparkLowLevel.h>

 
using namespace frc2::cmd;

SubIntake::SubIntake() {
  if (BotVars::activeRobot == BotVars::PRACTICE) {
    _intakeMotor = std::make_unique<ICSparkMax>(canid::IntakeMotor, CURRENT_LIMIT);
  } else {
    _intakeMotor = std::make_unique<ICSparkFlex>(canid::IntakeMotor, CURRENT_LIMIT);
  }

  _intakeMotor->_spark->SetInverted(false);
  _intakeMotor->_spark->BurnFlash();
}

frc2::CommandPtr SubIntake::Outtake() {
  return Run([this]{ _intakeMotor->_spark->Set(-1);}).FinallyDo([this]{_intakeMotor->_spark->Set(0);});
}

frc2::CommandPtr SubIntake::Intake() {
return Run([this]{ _intakeMotor->_spark->Set(1);}).FinallyDo([this]{_intakeMotor->_spark->Set(0);});
}


// This method will be called once per scheduler run
void SubIntake::Periodic() {
  frc::SmartDashboard::PutNumber("Intake/Speed", _intakeMotor->_spark->Get());
}
