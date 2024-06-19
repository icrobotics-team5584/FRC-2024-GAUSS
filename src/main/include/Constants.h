#pragma once 


namespace canid{
    constexpr int ShooterPivotMotor  = 13;
    constexpr int ShooterPivotEncoder  = 14;
    constexpr int ShooterFlywheelMotorLeft  = 15;
    constexpr int ShooterFlywheelMotorRight  = 16;

    constexpr int IntakeMotor  = 17;

    constexpr int FeederMotor = 20;
    
    constexpr int ClimberMain = 18;
    constexpr int ClimberSecond = 19;

    constexpr int DriveBaseFrontRightDrive = 1;
    constexpr int DriveBaseFrontRightTurn = 2;
    constexpr int DriveBaseFrontRightEncoder = 3;

    constexpr int DriveBaseFrontLeftDrive = 4;
    constexpr int DriveBaseFrontLeftTurn = 5;
    constexpr int DriveBaseFrontLeftEncoder = 6;
        
    constexpr int DriveBaseBackRightDrive = 7;
    constexpr int DriveBaseBackRightTurn = 8;
    constexpr int DriveBaseBackRightEncoder = 9;

    constexpr int DriveBaseBackLeftDrive = 10;
    constexpr int DriveBaseBackLeftTurn = 11;
    constexpr int DriveBaseBackLeftEncoder = 12;
}

namespace dio{
constexpr int ShooterLineBreak = 1;
}