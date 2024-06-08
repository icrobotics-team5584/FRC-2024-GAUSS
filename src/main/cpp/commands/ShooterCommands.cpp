// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShooterCommands.h"
#include "subsystems/SubFeeder.h"
#include "subsystems/SubPivot.h"
#include "subsystems/SubIntake.h"
#include "subsystems/SubShooter.h"
#include "subsystems/SubVision.h"
#include "stdio.h"
#include "iostream"

namespace cmd {
using namespace frc2::cmd;
frc2::CommandPtr CmdIntake(){
    return SubIntake::GetInstance().Intake().AlongWith(SubFeeder::GetInstance().FeedToShooter())
    .Until([]{return SubFeeder::GetInstance().CheckHasNote();});
}
frc2::CommandPtr CmdOuttake(){
    return SubIntake::GetInstance().Outtake().AlongWith(SubFeeder::GetInstance().FeedToIntake());
}
frc2::CommandPtr CmdShootSpeaker(){
    return Parallel(
        SubPivot::GetInstance().CmdPivotFromVision([]{return SubVision::GetInstance().GetTagPitch();}),
        SubShooter::GetInstance().CmdSetShooterSpeaker(),
        Sequence(
            WaitUntil([]{return SubPivot::GetInstance().IsOnTarget();}),
            WaitUntil([]{return SubShooter::GetInstance().IsOnTarget();}),
            SubFeeder::GetInstance().FeedToShooter()
        )
    )
    .FinallyDo([] {SubShooter::GetInstance().CmdSetShooterOff();});
}
frc2::CommandPtr CmdShootAmp(){
    return Parallel(
        SubPivot::GetInstance().CmdSetPivotAngle(90_deg),
        SubShooter::GetInstance().CmdSetShooterAmp(),
        SubFeeder::GetInstance().FeedToIntake()
    )
    //.Until([] {return !SubFeeder::GetInstance().GetFeederState();})
    .Until([] {return false;})
    .FinallyDo([] {SubShooter::GetInstance().CmdSetShooterOff();});
}
frc2::CommandPtr CmdShootPassing(){
    return Parallel(
        SubPivot::GetInstance().CmdSetPivotAngle(30_deg),
        SubShooter::GetInstance().CmdSetShooterPassing(),
        SubFeeder::GetInstance().FeedToIntake()
    )
    //.Until([] {return !SubFeeder::GetInstance().GetFeederState();})
    .Until([] {return false;})
    .FinallyDo([] {SubShooter::GetInstance().CmdSetShooterOff();});
}

frc2::CommandPtr CmdShootNeutral() {
    return SubShooter::GetInstance().CmdSetShooterSpeaker();
}
}