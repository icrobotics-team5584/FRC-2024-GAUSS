// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubPivot.h"
#include <frc2/command/commands.h>
#include <frc/smartdashboard/SmartDashboard.h>


SubPivot::SubPivot(){
    _pivotMotor.SetPIDFF(_pivotP, _pivotI, _pivotD);

    frc::SmartDashboard::PutData("Pivot/Motor", (wpi::Sendable*)&_pivotMotor);
}

// This method will be called once per scheduler run
void SubPivot::Periodic() {}

frc2::CommandPtr SubPivot::CmdSetPivotAngle(units::degree_t targetAngle){
    return RunOnce([this, targetAngle]{
        _pivotMotor.SetPositionTarget(targetAngle);
    });
}

void SubPivot::SimulationPeriodic(){
    _pivotSim.SetInputVoltage(_pivotMotor.GetSimVoltage());
    _pivotSim.Update(20_ms);

    _pivotMotor.UpdateSimEncoder(_pivotSim.GetAngle(), _pivotSim.GetVelocity());
}