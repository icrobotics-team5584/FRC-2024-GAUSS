// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubShooter.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "utilities/RobotLogs.h"


using namespace frc2::cmd;

SubShooter::SubShooter(){
    ctre::phoenix6::configs::TalonFXConfiguration flywheelConfig{};
    flywheelConfig.Slot0.kP = _flywheelP;
    flywheelConfig.Slot0.kI = _flywheelI;
    flywheelConfig.Slot0.kD = _flywheelD;
    flywheelConfig.Slot0.kV = _flywheelV;
    flywheelConfig.Voltage.PeakForwardVoltage = 12;
    flywheelConfig.Voltage.PeakReverseVoltage = 0;
    flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    flywheelConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    flywheelConfig.CurrentLimits.SupplyCurrentThreshold = 50.0;
    flywheelConfig.CurrentLimits.SupplyTimeThreshold = 0.1;

    _ShooterFlywheelMotorRight.GetConfigurator().Apply(flywheelConfig);
    flywheelConfig.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
    _ShooterFlywheelMotorLeft.GetConfigurator().Apply(flywheelConfig);
}

// This method will be called once per scheduler run
void SubShooter::Periodic() {
frc::SmartDashboard::PutNumber("Shooter/SpeedLeft", _ShooterFlywheelMotorLeft.GetVelocity().GetValue().value());
frc::SmartDashboard::PutNumber("Shooter/SpeedRight", _ShooterFlywheelMotorRight.GetVelocity().GetValue().value());
frc::SmartDashboard::PutBoolean("Target/FlywheelOnTarget", IsOnTarget());
frc::SmartDashboard::PutNumber("Shooter/LeftCurrent", _ShooterFlywheelMotorLeft.GetStatorCurrent().GetValue().value());
frc::SmartDashboard::PutNumber("Shooter/RightCurrent", _ShooterFlywheelMotorRight.GetStatorCurrent().GetValue().value());
Logger::logFalcon(_ShooterFlywheelMotorLeft, "leftFlywheelMotor");
}

frc2::CommandPtr SubShooter::CmdSetShooterSpeaker(){
    return RunOnce([this]{
        _ShooterFlywheelMotorLeft.SetControl(_flywheelTargetVelocity.WithVelocity(SpeakerSpeedLeft));
        _ShooterFlywheelMotorRight.SetControl(_flywheelTargetVelocity.WithVelocity(SpeakerSpeedRight));
        });
}
frc2::CommandPtr SubShooter::CmdSetShooterAmp(){
    return RunOnce([this]{
        _ShooterFlywheelMotorLeft.SetControl(_flywheelTargetVelocity.WithVelocity(AmpSpeed));
        _ShooterFlywheelMotorRight.SetControl(_flywheelTargetVelocity.WithVelocity(AmpSpeed));
        });
}
frc2::CommandPtr SubShooter::CmdSetShooterPassing(){
    return RunOnce([this]{
        _ShooterFlywheelMotorLeft.SetControl(_flywheelTargetVelocity.WithVelocity(PassingSpeed));
        _ShooterFlywheelMotorRight.SetControl(_flywheelTargetVelocity.WithVelocity(PassingSpeed));
        });
}
frc2::CommandPtr SubShooter::CmdSetShooterOff(){
    return RunOnce([this]{
        _ShooterFlywheelMotorLeft.SetControl(_flywheelTargetVelocity.WithVelocity(ShooterOff));
        _ShooterFlywheelMotorRight.SetControl(_flywheelTargetVelocity.WithVelocity(ShooterOff));
        });
}
frc2::CommandPtr SubShooter::CmdSourcePickUpIntake(){
    return RunOnce([this]{
        _ShooterFlywheelMotorLeft.Set(-1);
        _ShooterFlywheelMotorRight.Set(-1);

    });
}

bool SubShooter::IsOnTarget() {
    auto tolerance = 1_tps;
    auto target = _flywheelTargetVelocity.Velocity;
    auto leftVelocity = _ShooterFlywheelMotorLeft.GetVelocity().GetValue();
    auto rightVelocity = _ShooterFlywheelMotorRight.GetVelocity().GetValue();
    if (
        // units::math::abs(target - leftVelocity) < tolerance
        units::math::abs(target - rightVelocity) < tolerance
        )
    {
        return true;
    }
    else
    {
        return false;
    }
}

void SubShooter::SimulationPeriodic() {
    auto& leftState = _ShooterFlywheelMotorLeft.GetSimState();
    _leftSim.SetInputVoltage(leftState.GetMotorVoltage());
    _leftSim.Update(20_ms);
    leftState.SetRawRotorPosition(_leftSim.GetAngularPosition());
    leftState.SetRotorVelocity(_leftSim.GetAngularVelocity());

    auto& rightState = _ShooterFlywheelMotorRight.GetSimState();
    _rightSim.SetInputVoltage(rightState.GetMotorVoltage());
    _rightSim.Update(20_ms);
    rightState.SetRawRotorPosition(_rightSim.GetAngularPosition());
    rightState.SetRotorVelocity(_rightSim.GetAngularVelocity());
}