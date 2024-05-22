#pragma once

#include "utilities/BotVars.h"

namespace canivore {
  // 1-12 inclusive

}

namespace canid {

  //drivebase IDS used: 1-12
 constexpr int DriveBaseFrontRightDrive = 7; //done
 constexpr int DriveBaseFrontRightTurn = 8;
 constexpr int DriveBaseFrontRightEncoder = 10; //done

 constexpr int DriveBaseFrontLeftDrive = 3;
 constexpr int DriveBaseFrontLeftTurn = 6; //6
 constexpr int DriveBaseFrontLeftEncoder = 9;
      
 constexpr int DriveBaseBackRightDrive = 5; //done
 constexpr int DriveBaseBackRightTurn = 2;
 constexpr int DriveBaseBackRightEncoder = 12; //done

 constexpr int DriveBaseBackLeftDrive = 1; //done
 constexpr int DriveBaseBackLeftTurn = 4;
 constexpr int DriveBaseBackLeftEncoder = 11; //done


//Intake IDS used: 13
  constexpr int IntakeMotor = 17;

//Shooter IDS used: 14-16
  constexpr int TopShooterMotor = 18;
  constexpr int BottomShooterMotor = 16;
  constexpr int ShooterFeederMotor = 14;

//Arm IDS used: 17-18
  constexpr int ArmMotor = 19; //19

//Amp/Trap IDS used: 19
  constexpr int AmpMotor = 20;

//Climber IDS used: 20-21
  constexpr int lClimbMotor = 15;
  constexpr int rClimbMotor = 13;

  constexpr int ClimberLeftLaserCAN = 21;
  constexpr int ClimberRightLaserCAN = 99; //same ID as Pcm1D

  constexpr int SpareSpark1 = 58;
  constexpr int SpareSpark2 = 60;

}

namespace pcm1 {
     //Intake
    constexpr int IntakeExtend = 0;
    constexpr int IntakeRetract = 1;

    //Shooter
    const int ShootClose = BotVars::Choose(6,3);
    const int ShootFar = BotVars::Choose(3,2); // change both bots to be the same if we have time

    //Climber
    constexpr int LockCylinderForward = 4;
    constexpr int LockCylinderReverse = 5;

    constexpr int Pcm1Id = 22; //same ID as ClimberRightLaserCan

}

namespace pwm {
  constexpr int LEDS = 0;
}


namespace OperatorConstants {}

constexpr int kDriverControllerPort = 0;

namespace dio {
  constexpr int FDLineBreak = 3;
  constexpr int SDLineBreak = 1;
  constexpr int ShooterLineBreak = 4;
  constexpr int IntakeExtendedReed = 0;
  constexpr int TopShooterEncoderChannelA = 8;
  constexpr int TopShooterEncoderChannelB = 9;
  constexpr int BottomShooterEncoderChannelA = 6;
  constexpr int BottomShooterEncoderChannelB = 7;
}