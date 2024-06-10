// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/commands.h>

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
  frc2::CommandPtr CmdShootSpeaker();
  frc2::CommandPtr CmdShootAmp();
  frc2::CommandPtr CmdShootPassing();
  frc2::CommandPtr CmdShootNeutral();
  frc2::CommandPtr CmdFeedOnceOnTarget();
}