#pragma once 


namespace canid{
    constexpr int ShooterPivotMotor  = 13;
    constexpr int ShooterFlywheelMotorLeft  = 14;
    constexpr int ShooterFlywheelMotorRight  = 15;
    constexpr int IntakeMotor  = 16;
    constexpr int FeederMotor = 17;

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


    
}

namespace dio{
    constexpr int IntakeLineBreak = 1;

    constexpr int FeederPointSwitch = 1;


}