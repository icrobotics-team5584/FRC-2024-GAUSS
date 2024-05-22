#pragma once 


namespace canid{
    constexpr int ShooterPivotMotor  = 9;
    constexpr int ShooterFlywheelMotorLeft  = 10;
    constexpr int ShooterFlywheelMotorRight  = 11;
    constexpr int IntakeMotor  = 12;

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
    constexpr int ShooterFlywheelEncoderLeftChannelA = 0;
    constexpr int ShooterFlywheelEncoderLeftChannelB = 1;
    constexpr int ShooterFlywheelEncoderRightChannelA = 2;
    constexpr int ShooterFlywheelEncoderRightChannelB = 3;
    
}