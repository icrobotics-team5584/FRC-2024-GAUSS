// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubPivot.h"
#include <frc2/command/commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>


SubPivot::SubPivot(){
    // Config cancoder
    ctre::phoenix6::configs::CANcoderConfiguration canCoderConfig{};
    canCoderConfig.MagnetSensor.MagnetOffset = 0.228271421875_tr;
    _shooterPivotEncoder.GetConfigurator().Apply(canCoderConfig);

    // Config pivot motor
    rev::spark::SparkBaseConfig pivotMotorConfig{};
    pivotMotorConfig
        .Inverted(true)
        .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
    pivotMotorConfig.closedLoop
        .P(_pivotP)
        .I(_pivotI)
        .D(_pivotD);
    pivotMotorConfig.closedLoop.maxMotion
        .MaxVelocity(0.2)
        .MaxAcceleration(1)
        .AllowedClosedLoopError(0);
    pivotMotorConfig.softLimit
        .ForwardSoftLimit(HIGH_STOP.value())
        .ReverseSoftLimit(LOW_STOP.value())
        .ForwardSoftLimitEnabled(true)
        .ReverseSoftLimitEnabled(true);
    pivotMotorConfig.encoder
        .PositionConversionFactor(1.0 / PIVOT_GEAR_RATIO)
        .VelocityConversionFactor(PIVOT_GEAR_RATIO / 60); // divide by 60 to turn RPM to tps
    _pivotMotor.AdjustConfig(pivotMotorConfig);

    _pivotMotor.SetFeedforwardGains(PIVOT_S, PIVOT_G, true, PIVOT_V, PIVOT_A);
    _pivotMotor.SetPosition(_shooterPivotEncoder.GetPosition().GetValue());

    frc::SmartDashboard::PutData("Pivot/Motor", &_pivotMotor);

    // Setup shooter pitch table
    _pitchTable.insert(-12_deg, 13.75_deg);
    _pitchTable.insert(-11_deg, 14.5_deg);
    _pitchTable.insert(-10_deg, 15.5_deg);
    _pitchTable.insert(-9_deg, 17_deg);
    _pitchTable.insert(-4.5_deg, 22.5_deg);
    _pitchTable.insert(0_deg, 27_deg);
    _pitchTable.insert(9_deg, 33_deg);
    _pitchTable.insert(10_deg, 34.5_deg);
}



// This method will be called once per scheduler run
void SubPivot::Periodic() {
    frc::SmartDashboard::PutString("Pivot/CurrentCommand", (GetCurrentCommand()->GetName()));
    frc::SmartDashboard::PutBoolean("Target/PivotOnTarget", IsOnTarget());
    _pivotMotor.UpdateControls();
}

frc2::CommandPtr SubPivot::CmdSetPivotAngle(units::degree_t targetAngle){
    return RunOnce([this, targetAngle]{
        _pivotMotor.SetMotionProfileTarget(targetAngle);
    }).WithName("SetPivotAngle");
}

frc2::CommandPtr SubPivot::CmdPivotFromVision(std::function<units::degree_t()> tagAngle){
    return Run([this, tagAngle]{
        _pivotMotor.SetPositionTarget(_pitchTable[tagAngle()], _pivotFF.Calculate(_pitchTable[tagAngle()], 0_tps));
        frc::SmartDashboard::PutNumber("Pivot/TagAngle", tagAngle().value());
    }).WithName("PivotFromVision");
}

void SubPivot::SimulationPeriodic(){
    _pivotSim.SetInputVoltage(_pivotMotor.CalcSimVoltage());
    _pivotSim.Update(20_ms);
    auto vel = _pivotSim.GetVelocity();
    _pivotMotor.IterateSim(vel);
}

bool SubPivot::IsOnTarget() {
    auto tolerance = 5_deg;
    return units::math::abs( _pivotMotor.GetPosError()) < tolerance;
}

units::turn_t SubPivot::GetAngle() {
    return _pivotMotor.GetPosition();
}

units::volt_t SubPivot::GetVoltage() {
    return _pivotMotor.GetMotorVoltage();
}