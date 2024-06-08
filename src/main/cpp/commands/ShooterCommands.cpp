// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShooterCommands.h"
#include "subsystems/SubFeeder.h"
#include "subsystems/SubPivot.h"
#include "subsystems/SubIntake.h"
#include "subsystems/SubShooter.h"
#include "stdio.h"
#include "iostream"

namespace cmd {
using namespace frc2::cmd;
frc2::CommandPtr CmdIntake(){
    return SubIntake::GetInstance().Intake().AlongWith(SubFeeder::GetInstance().FeedToIntake())
    .Until([]{return SubFeeder::GetInstance().GetFeederState();});
}
frc2::CommandPtr CmdShootSpeaker(){
    return Parallel(
        SubPivot::GetInstance().CmdSetPivotAngle(40_deg),
        SubShooter::GetInstance().CmdSetShooterSpeaker(),
        SubFeeder::GetInstance().FeedToIntake()
    )
    //.Until([] {return !SubFeeder::GetInstance().GetFeederState();})
    .Until([] {return false;})
    .FinallyDo([] {SubShooter::GetInstance().CmdSetShooterOff();});
}
//DO WHAT WAS DONE TO AMP TO EVERYTHING ELSE!
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