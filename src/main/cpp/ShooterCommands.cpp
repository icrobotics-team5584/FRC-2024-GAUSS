// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ShooterCommands.h"
#include "subsystems/SubFeeder.h"
#include "subsystems/SubPivot.h"
#include "subsystems/SubIntake.h"
#include "subsystems/SubShooter.h"

namespace cmd {
using namespace frc2::cmd;
frc2::CommandPtr CmdIntake(){
    return SubIntake::GetInstance().Intake().AlongWith(SubFeeder::GetInstance().FeedToIntake())
    .Until([]{return SubFeeder::GetInstance().GetFeederState();});
}
frc2::CommandPtr CmdShootSpeaker(){
    return Run([]{
        SubShooter::GetInstance().CmdSetShooterSpeaker();
        SubFeeder::GetInstance().FeedToIntake();
    })
    .Until([] {return !SubFeeder::GetInstance().GetFeederState();})
    .FinallyDo([] {SubShooter::GetInstance().CmdSetShooterOff();});
}
frc2::CommandPtr CmdShootAmp(){
    return Run([]{
        SubShooter::GetInstance().CmdSetShooterAmp();
        SubFeeder::GetInstance().FeedToIntake();
    })
    .Until([] {return !SubFeeder::GetInstance().GetFeederState();})
    .FinallyDo([] {SubShooter::GetInstance().CmdSetShooterOff();});
}
frc2::CommandPtr CmdShootPassing(){
    return Run([]{
        SubShooter::GetInstance().CmdSetShooterPassing();
        SubFeeder::GetInstance().FeedToIntake();
    })
    .Until([] {return !SubFeeder::GetInstance().GetFeederState();})
    .FinallyDo([] {SubShooter::GetInstance().CmdSetShooterOff();});
}
}