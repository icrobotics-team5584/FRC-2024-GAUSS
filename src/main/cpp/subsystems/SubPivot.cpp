// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubPivot.h"
#include <frc2/command/commands.h>
#include <frc/smartdashboard/SmartDashboard.h>


SubPivot::SubPivot(){
    _pivotMotor.SetPIDFF(_pivotP, _pivotI, _pivotD);

    frc::SmartDashboard::PutData("Pivot/Motor", (wpi::Sendable*)&_pivotMotor);

    //Setup shooter pitch table
    _pitchTable.insert(64_deg, 90_deg);
    _pitchTable.insert(32_deg, 84_deg);
    _pitchTable.insert(16_deg, 70_deg);
    _pitchTable.insert(8_deg, 58_deg);
    _pitchTable.insert(4_deg, 49_deg);
    _pitchTable.insert(2_deg, 45_deg);
}



// This method will be called once per scheduler run
void SubPivot::Periodic() {
    frc::SmartDashboard::PutString("Pivot/CurrentCommand", (GetCurrentCommand()->GetName()));
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