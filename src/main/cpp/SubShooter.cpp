// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubShooter.h"

using namespace frc2::cmd;

SubShooter::SubShooter(){
    _leftEncoder.SetSamplesToAverage(30);
    _leftEncoder.SetDistancePerPulse(1.00/2048.00);
    _rightEncoder.SetSamplesToAverage(30);
    _rightEncoder.SetDistancePerPulse(1.00/2048.00);
}

// This method will be called once per scheduler run
void SubShooter::Periodic() {
    _leftEncoderDif = (_leftEncoder.GetDistance() - _leftEncoderPositionPrev) / 0.02;
    _rightEncoderDif = (_rightEncoder.GetDistance() - _rightEncoderPositionPrev) / 0.02;
    _leftEncoderPositionPrev = _leftEncoder.GetDistance();
    _rightEncoderPositionPrev = _rightEncoder.GetDistance();
    
    _leftPastVelocityMeasurements[2] = _leftPastVelocityMeasurements[1];
    _leftPastVelocityMeasurements[1] = _leftPastVelocityMeasurements[0];
    _leftPastVelocityMeasurements[0] = _leftEncoderDif;

    _rightPastVelocityMeasurements[2] = _rightPastVelocityMeasurements[1];
    _rightPastVelocityMeasurements[1] = _rightPastVelocityMeasurements[0];
    _rightPastVelocityMeasurements[0] = _rightEncoderDif;

    _leftPastVelocityAverage = (_leftPastVelocityMeasurements[2] + _leftPastVelocityMeasurements[1] + _leftPastVelocityMeasurements[0]) / 3;
    _rightPastVelocityAverage = (_rightPastVelocityMeasurements[2] + _rightPastVelocityMeasurements[1] + _rightPastVelocityMeasurements[0]) / 3;

    
}

void SubShooter::UpdatePIDFF(){
    auto FFVolts = _shooterFeedForward.Calculate(CurrentShooterTarget);
    auto LeftVolts = _leftShooterPID.Calculate(_leftPastVelocityAverage, CurrentShooterTarget.value())*1_V + FFVolts;
    if (LeftVolts < 0_V){
        LeftVolts = 0_V;
    }
    _shooterFlywheelMotorLeft.SetVoltage(LeftVolts);

    auto RightVolts = _rightShooterPID.Calculate(_rightPastVelocityAverage, CurrentShooterTarget.value())*1_V + FFVolts;
    if (RightVolts < 0_V){
        RightVolts = 0_V;
    }
    _shooterFlywheelMotorRight.SetVoltage(RightVolts);
}
