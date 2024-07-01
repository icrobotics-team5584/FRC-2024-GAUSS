// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubPivot.h"
#include <frc2/command/commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>


SubPivot::SubPivot(){
    ctre::phoenix6::configs::CANcoderConfiguration pivotConfig{};
    pivotConfig.MagnetSensor.MagnetOffset = 0.5164954444444444;

    _shooterPivotEncoder.GetConfigurator().Apply(pivotConfig);

    _pivotMotor.SetPIDFF(_pivotP, _pivotI, _pivotD);
    _pivotMotor.SetConversionFactor(1/PIVOT_GEAR_RATIO);
    _pivotMotor.SetPosition(_shooterPivotEncoder.GetPosition().GetValue());
    _pivotMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);

    frc::SmartDashboard::PutData("Pivot/Motor", (wpi::Sendable*)&_pivotMotor);

    //Setup shooter pitch table
    _pitchTable.insert(-20_deg, 65_deg);
    _pitchTable.insert(-15_deg, 55_deg);
    _pitchTable.insert(-9_deg, 40_deg);
    _pitchTable.insert(0_deg, 28_deg);
    _pitchTable.insert(9_deg, 20_deg);
    _pitchTable.insert(17_deg, 15_deg);
}



// This method will be called once per scheduler run
void SubPivot::Periodic() {
    frc::SmartDashboard::PutString("Pivot/CurrentCommand", (GetCurrentCommand()->GetName()));
    frc::SmartDashboard::PutBoolean("Target/PivotOnTarget", IsOnTarget());
}

frc2::CommandPtr SubPivot::CmdSetPivotAngle(units::degree_t targetAngle){
    return RunOnce([this, targetAngle]{
        _pivotMotor.SetPositionTarget(targetAngle, _pivotFF.Calculate(targetAngle, 0_tps));
    }).WithName("SetPivotAngle");
}

frc2::CommandPtr SubPivot::CmdPivotFromVision(std::function<units::degree_t()> tagAngle){
    return Run([this, tagAngle]{
        _pivotMotor.SetPositionTarget(_pitchTable[tagAngle()], _pivotFF.Calculate(_pitchTable[tagAngle()], 0_tps));
        frc::SmartDashboard::PutNumber("Pivot/TagAngle", tagAngle().value());
    }).WithName("PivotFromVision");
}

void SubPivot::SimulationPeriodic(){
    _pivotSim.SetInputVoltage(_pivotMotor.GetSimVoltage());
    _pivotSim.Update(20_ms);

    _pivotMotor.UpdateSimEncoder(_pivotSim.GetAngle(), _pivotSim.GetVelocity());
}

bool SubPivot::IsOnTarget() {
    auto tolerance = 5_deg;
    return units::math::abs( _pivotMotor.GetPosError()) < tolerance;
}