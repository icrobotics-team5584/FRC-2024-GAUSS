// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include "Constants.h"
#include <frc/DigitalInput.h>

class SubIntake : public frc2::SubsystemBase {
 public:
  SubIntake();

  static SubIntake& GetInstance() {
    static SubIntake inst;
    return inst;
  }
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */

//Define commands for intake
  frc2::CommandPtr Intake();
  frc2::CommandPtr Outtake();
  //Defind functions for intake
  void Periodic() override;


 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  rev::CANSparkMax _intakeMotor{canid::IntakeMotor, rev::CANSparkMax::MotorType::kBrushless};
};
