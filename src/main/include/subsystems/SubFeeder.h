// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "utilities/ICSparkMax.h"
#include "Constants.h"
#include <frc/DigitalInput.h>

class SubFeeder : public frc2::SubsystemBase {
 public:
  SubFeeder();

  frc2::CommandPtr FeedToShooter();
  frc2::CommandPtr FeedToIntake();

  bool GetFeederState();
  
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  //motors
  ICSparkMax _feederMotor{canid::FeederMotor, 40_A};

  frc::DigitalInput _feederPointSwitch{dio::FeederPointSwitch};
};