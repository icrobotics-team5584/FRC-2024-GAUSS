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
    .Until([]{return SubFeeder::GetInstance().CheckHasNote();})
    .AndThen(SubFeeder::GetInstance().StopFeeder());
}

frc2::CommandPtr CmdFeedOnceOnTarget() {
    return Sequence(
            WaitUntil([]{return SubPivot::GetInstance().IsOnTarget();}),
            WaitUntil([]{return SubShooter::GetInstance().IsOnTarget();}),
            WaitUntil([]{return SubVision::GetInstance().IsFacingTarget();}),
            SubFeeder::GetInstance().FeedToShooter()
        );
}
frc2::CommandPtr CmdFeedOnceOnAmpTarget() {
    return Sequence(
            WaitUntil([]{return SubPivot::GetInstance().IsOnTarget();}),
            WaitUntil([]{return SubShooter::GetInstance().IsOnTarget();}),
            SubFeeder::GetInstance().FeedToShooter()
        );
}

frc2::CommandPtr CmdOuttake(){
    return SubIntake::GetInstance().Outtake().AlongWith(SubFeeder::GetInstance().FeedToIntake());
}

frc2::CommandPtr CmdShootSpeaker(frc2::CommandXboxController& controller){
    return Parallel(
        SubPivot::GetInstance().CmdPivotFromVision([]{    /*default value = 60 degrees(Subwoofer shot)*/
            return SubVision::GetInstance().GetLatestSpeakerPitch().value_or(60_deg);}),
        SubShooter::GetInstance().CmdSetShooterSpeaker(),
        CmdAimAtSpeakerWithVision(controller),
        CmdFeedOnceOnTarget()
    )
    .FinallyDo([] {SubShooter::GetInstance().CmdSetShooterOff();});
}

frc2::CommandPtr CmdShootAmp(){
    return Parallel(
        SubPivot::GetInstance().CmdSetPivotAngle(40_deg),
        SubShooter::GetInstance().CmdSetShooterAmp(),
        CmdFeedOnceOnAmpTarget()
    )
    .Until([] {return !SubFeeder::GetInstance().CheckHasNote();})
    .FinallyDo([] {SubShooter::GetInstance().CmdSetShooterOff();});
}

frc2::CommandPtr CmdShootPassing(){
    return Parallel(
        SubPivot::GetInstance().CmdSetPivotAngle(35_deg),
        SubShooter::GetInstance().CmdSetShooterPassing(),
        CmdFeedOnceOnTarget()
    )
    .Until([] {return !SubFeeder::GetInstance().CheckHasNote();})
    .FinallyDo([] {SubShooter::GetInstance().CmdSetShooterOff();});
}

frc2::CommandPtr CmdShootNeutral() {
    return SubShooter::GetInstance().CmdSetShooterSpeaker();
}

frc2::CommandPtr CmdShootSubwoofer() {
    return Parallel(
        SubPivot::GetInstance().CmdSetPivotAngle(55_deg),
        SubShooter::GetInstance().CmdSetShooterSpeaker(),
        CmdFeedOnceOnTarget()
        )
        .Until([] {return !SubFeeder::GetInstance().CheckHasNote();})
        .FinallyDo([] {SubShooter::GetInstance().CmdSetShooterOff();});
}

frc2::CommandPtr CmdAimAtSpeakerWithVision(frc2::CommandXboxController& controller){
  static units::degree_t camYaw = 0_deg;
  static units::degree_t startingGyroYaw = 0_deg;

  return RunOnce([] {camYaw = 0_deg;
    startingGyroYaw = SubDrivebase::GetInstance().GetHeading().Degrees(); })
      .AndThen(Run([] {
        auto result = SubVision::GetInstance().GetSpeakerYaw();

        if (result.has_value()) {
          camYaw = SubVision::GetInstance().GetSpeakerYaw().value_or(0_deg);
          startingGyroYaw = SubDrivebase::GetInstance().GetHeading().Degrees();
        }

        units::degree_t currentGyroYaw = SubDrivebase::GetInstance().GetHeading().Degrees();
        units::degree_t gyroAngleTravelled = currentGyroYaw - startingGyroYaw;
        units::degree_t errorAngle = -camYaw - gyroAngleTravelled;
        frc::SmartDashboard::PutNumber("Vision/Result", result.value_or(0_deg).value());
        frc::SmartDashboard::PutNumber("Vision/currentGyroYaw ", currentGyroYaw.value());
        frc::SmartDashboard::PutNumber("Vision/startingGyroYaw ", startingGyroYaw.value());
        frc::SmartDashboard::PutNumber("Vision/GyroAngleTravelled ", gyroAngleTravelled.value());
        frc::SmartDashboard::PutNumber("Vision/ErrorAngle ", errorAngle.value());
        
        SubDrivebase::GetInstance().RotateToZero(-errorAngle);
        
      }))
      .AlongWith(SubDrivebase::GetInstance().JoystickDrive(controller, true))
      .FinallyDo([]{SubDrivebase::GetInstance().StopDriving();});
}

frc2::CommandPtr CmdAimWithoutControl(){ // For auto
  static units::degree_t camYaw = 0_deg;
  static units::degree_t startingGyroYaw = 0_deg;

  return RunOnce([] {camYaw = 0_deg;
    startingGyroYaw = SubDrivebase::GetInstance().GetHeading().Degrees(); })
      .AndThen(Run([] {
        auto result = SubVision::GetInstance().GetSpeakerYaw();

        if (result.has_value()) {
          camYaw = SubVision::GetInstance().GetSpeakerYaw().value_or(0_deg);
          startingGyroYaw = SubDrivebase::GetInstance().GetHeading().Degrees();
        }

        units::degree_t currentGyroYaw = SubDrivebase::GetInstance().GetHeading().Degrees();
        units::degree_t gyroAngleTravelled = currentGyroYaw - startingGyroYaw;
        units::degree_t errorAngle = camYaw - gyroAngleTravelled;
        frc::SmartDashboard::PutNumber("Vision/Result", result.value_or(0_deg).value());
        frc::SmartDashboard::PutNumber("Vision/currentGyroYaw ", currentGyroYaw.value());
        frc::SmartDashboard::PutNumber("Vision/startingGyroYaw ", startingGyroYaw.value());
        frc::SmartDashboard::PutNumber("Vision/GyroAngleTravelled ", gyroAngleTravelled.value());
        frc::SmartDashboard::PutNumber("Vision/ErrorAngle ", errorAngle.value());
        
        SubDrivebase::GetInstance().RotateToZero(-errorAngle);
        
      }))
      .FinallyDo([]{SubDrivebase::GetInstance().StopDriving();});
}

frc2::CommandPtr CmdShootSpeakerAuto() {
    if (frc::RobotBase::IsSimulation() == true) {
        return Sequence(
            SubShooter::GetInstance().CmdSetShooterSpeaker(),
            Wait(0.5_s),
            SubShooter::GetInstance().CmdSetShooterOff()
        );
    }
    else {
        return Parallel(
            SubPivot::GetInstance().CmdPivotFromVision([]{    /*default value = 60 degrees(Subwoofer shot)*/
                return SubVision::GetInstance().GetSpeakerPitch().value_or(60_deg);}).WithTimeout(1_s),
            SubShooter::GetInstance().CmdSetShooterSpeaker(),
            CmdAimWithoutControl().WithTimeout(1_s),
            CmdFeedOnceOnTarget().WithTimeout(1_s).AndThen(SubFeeder::GetInstance().FeedToShooter().WithTimeout(1_s))
        )
        .FinallyDo([] {SubShooter::GetInstance().CmdSetShooterOff();});
    }
}
}