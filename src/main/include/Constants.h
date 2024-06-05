#pragma once 


namespace canid{
    constexpr int ShooterPivotMotor  = 13;
    constexpr int ShooterFlywheelMotorLeft  = 14;
    constexpr int ShooterFlywheelMotorRight  = 15;
    constexpr int IntakeMotor  = 16;
    constexpr int FeederMotor = 17;

    constexpr int DriveBaseFrontRightDrive = 1; //done
    constexpr int DriveBaseFrontRightTurn = 2;
    constexpr int DriveBaseFrontRightEncoder = 3; //done

    constexpr int DriveBaseFrontLeftDrive = 4;
    constexpr int DriveBaseFrontLeftTurn = 5; //6
    constexpr int DriveBaseFrontLeftEncoder = 6;
        
    constexpr int DriveBaseBackRightDrive = 7; //done
    constexpr int DriveBaseBackRightTurn = 8;
    constexpr int DriveBaseBackRightEncoder = 9; //done

    constexpr int DriveBaseBackLeftDrive = 10; //done
    constexpr int DriveBaseBackLeftTurn = 11;
    constexpr int DriveBaseBackLeftEncoder = 12; //done

    constexpr int ClimberMain = 17;
    constexpr int ClimberSecond = 18;


    
}

namespace dio{
constexpr int ShooterLineBreak = 1;
}