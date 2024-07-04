// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/commands.h>
#include <frc2/command/button/CommandXboxController.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
namespace cmd {
  frc2::CommandPtr CmdIntake();
  frc2::CommandPtr CmdOuttake();
  frc2::CommandPtr CmdShootSpeaker(frc2::CommandXboxController& controller);
  frc2::CommandPtr CmdShootSpeakerAuto();
  frc2::CommandPtr CmdShootAmp();
  frc2::CommandPtr CmdShootPassing();
  frc2::CommandPtr CmdShootNeutral();
  frc2::CommandPtr CmdFeedOnceOnTarget();
  frc2::CommandPtr CmdShootSubwoofer();
  frc2::CommandPtr CmdAimAtSpeakerWithVision(frc2::CommandXboxController& controller);
  frc2::CommandPtr CmdAimWithoutControl();
}