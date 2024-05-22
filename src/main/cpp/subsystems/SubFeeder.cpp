// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubFeeder.h"
#include <frc2/command/commands.h>

SubFeeder::SubFeeder() = default;

frc2::CommandPtr SubFeeder::FeedToIntake() {
    return StartEnd([this] {_feederMotor.Set(1);},
                    [this] {_feederMotor.Set(1);});
}
// This method will be called once per scheduler run
void SubFeeder::Periodic() {}
