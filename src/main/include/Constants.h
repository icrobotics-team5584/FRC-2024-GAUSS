#pragma once 


namespace canid{
    constexpr int ShooterPivotMotor  = 9;
    constexpr int ShooterFlywheelMotorLeft  = 10;
    constexpr int ShooterFlywheelMotorRight  = 11;
    constexpr int IntakeMotor  = 12;
    constexpr int FeederMotor = 17;

    constexpr int DriveBaseFrontRightDrive = 1; //done
    constexpr int DriveBaseFrontRightTurn = 2;
    constexpr int DriveBaseFrontRightEncoder = 15; //done

    constexpr int DriveBaseFrontLeftDrive = 3;
    constexpr int DriveBaseFrontLeftTurn = 4; //6
    constexpr int DriveBaseFrontLeftEncoder = 16;
        
    constexpr int DriveBaseBackRightDrive = 5; //done
    constexpr int DriveBaseBackRightTurn = 6;
    constexpr int DriveBaseBackRightEncoder = 17; //done

    constexpr int DriveBaseBackLeftDrive = 7; //done
    constexpr int DriveBaseBackLeftTurn = 8;
    constexpr int DriveBaseBackLeftEncoder = 18; //done

    constexpr int ClimberMain = 13;
    constexpr int ClimberSecond = 14;


    
}

namespace dio{
constexpr int ShooterLineBreak = 1;
}