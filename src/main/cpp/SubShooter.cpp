// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubShooter.h"
#include "frc/smartdashboard/SmartDashboard.h"

using namespace frc2::cmd;

SubShooter::SubShooter(){
    ctre::phoenix6::configs::TalonFXConfiguration flywheelConfig{};
    flywheelConfig.Slot0.kP = _flywheelP;
    flywheelConfig.Slot0.kI = _flywheelI;
    flywheelConfig.Slot0.kD = _flywheelD;
    flywheelConfig.Slot0.kV = _flywheelV;
    flywheelConfig.Voltage.PeakForwardVoltage = 12;
    flywheelConfig.Voltage.PeakReverseVoltage = 0;

    _ShooterFlywheelMotorLeft.GetConfigurator().Apply(flywheelConfig);
    _ShooterFlywheelMotorRight.GetConfigurator().Apply(flywheelConfig);
}

// This method will be called once per scheduler run
void SubShooter::Periodic() {
frc::SmartDashboard::PutNumber("ShooterSpeed", _ShooterFlywheelMotorLeft.GetVelocity().GetValue().value());
frc::SmartDashboard::PutNumber("ShooterTargetSpeed", _ShooterFlywheelMotorLeft.GetPIDVelocity_Reference().GetValue().value());
}

frc2::CommandPtr SubShooter::CmdSetShooterSpeaker(){
    return RunOnce([this]{
        _ShooterFlywheelMotorLeft.SetControl(_flywheelVelocity.WithVelocity(SpeakerSpeed));
        _ShooterFlywheelMotorRight.SetControl(_flywheelVelocity.WithVelocity(SpeakerSpeed));
        });
}
frc2::CommandPtr SubShooter::CmdSetShooterAmp(){
    return RunOnce([this]{
        _ShooterFlywheelMotorLeft.SetControl(_flywheelVelocity.WithVelocity(AmpSpeed));
        _ShooterFlywheelMotorRight.SetControl(_flywheelVelocity.WithVelocity(AmpSpeed));
        });
}
frc2::CommandPtr SubShooter::CmdSetShooterPassing(){
    return RunOnce([this]{
        _ShooterFlywheelMotorLeft.SetControl(_flywheelVelocity.WithVelocity(PassingSpeed));
        _ShooterFlywheelMotorRight.SetControl(_flywheelVelocity.WithVelocity(PassingSpeed));
        });
}
frc2::CommandPtr SubShooter::CmdSetShooterOff(){
    return RunOnce([this]{
        _ShooterFlywheelMotorLeft.SetControl(_flywheelVelocity.WithVelocity(ShooterOff));
        _ShooterFlywheelMotorRight.SetControl(_flywheelVelocity.WithVelocity(ShooterOff));
        });
}