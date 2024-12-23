// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubClimber.h"
#include "subsystems/SubIntake.h"
#include <frc/RobotBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>

SubClimber::SubClimber() {
    // Make common config for left and right
    rev::spark::SparkBaseConfig config;
    config
        .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
    config.softLimit
        .ForwardSoftLimit(DistanceToTurn(TOP_HEIGHT).value())
        .ReverseSoftLimit(DistanceToTurn(0_m).value());
    config.encoder
        .PositionConversionFactor(1.0 / gearRatio)
        .VelocityConversionFactor(gearRatio / 60); // divide by 60 to turn RPM to tps
    config.closedLoop.maxMotion
        .MaxVelocity(2)
        .MaxAcceleration(3)
        .AllowedClosedLoopError(0);

    //Set up left motor
    _lClimbMotor.SetFeedforwardGains(0_V, 0_V, false, 5.5_V/1_tps);
    config.closedLoop.P(lP).I(lI).D(lD);
    config.Inverted(false);
    _lClimbMotor.AdjustConfig(config);

    //Set up right motor
    _rClimbMotor.SetFeedforwardGains(0_V, 0_V, false, 5.5_V/1_tps);
    config.closedLoop.P(rP).I(rI).D(rD);
    config.Inverted(true);
    _rClimbMotor.AdjustConfig(config);

    //Enable top and bottom limit
    EnableSoftLimit(false);

    //Put motor data to dashboard
    frc::SmartDashboard::PutData("Climber/Left motor", &_lClimbMotor);
    frc::SmartDashboard::PutData("Climber/Right motor", &_rClimbMotor);
};

void SubClimber::Periodic() {
    frc::SmartDashboard::PutNumber("Climber/Left distance", TurnToDistance(_lClimbMotor.GetPosition()).value());
    frc::SmartDashboard::PutNumber("Climber/Right distance", TurnToDistance(_rClimbMotor.GetPosition()).value());
    frc::SmartDashboard::PutNumber("Climber/Target distance", TargetDistance.value());
    frc::SmartDashboard::PutNumber("Climber/Left current", _lClimbMotor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Climber/Right current", _rClimbMotor.GetOutputCurrent());
    frc::SmartDashboard::PutBoolean("Climber/Reseted", Reseted);
    frc::SmartDashboard::PutBoolean("Climber/Reseting", Reseting);
    _lClimbMotor.UpdateControls();
    _rClimbMotor.UpdateControls();
}

void SubClimber::SimulationPeriodic() {
    frc::SmartDashboard::PutData("Climber/Mech Display", &mech);
    frc::SmartDashboard::PutNumber("Climber/Left sim distance", TurnToDistance(_lClimbMotor.GetPosition()).value());
    frc::SmartDashboard::PutNumber("Climber/Right sim distance", TurnToDistance(_rClimbMotor.GetPosition()).value());

  lElvSim.SetInputVoltage(_lClimbMotor.CalcSimVoltage());
  lElvSim.Update(20_ms);
  _lClimbMotor.IterateSim(DistanceToTurn(lElvSim.GetVelocity()));

    rElvSim.SetInputVoltage(_rClimbMotor.CalcSimVoltage());
    rElvSim.Update(20_ms);
    _rClimbMotor.IterateSim(DistanceToTurn(rElvSim.GetVelocity()));

    mechLeftElevator->SetLength(TurnToDistance(_lClimbMotor.GetPosition()).value() * 4);
    mechRightElevator->SetLength(TurnToDistance(_rClimbMotor.GetPosition()).value() * 4);
    mechTar->SetLength(TargetDistance.value() * 4);
}

//Unit translation from meters to climber motor rotations(turns)
units::turn_t SubClimber::DistanceToTurn(units::meter_t distance) {
  return distance / WheelCir * 1_tr;
}

units::radians_per_second_t SubClimber::DistanceToTurn(units::meters_per_second_t distance) {
    return distance / WheelCir * 1_tr;
}

//Unti translation from climber motor rotations(turns) to meters
units::meter_t SubClimber::TurnToDistance(units::turn_t turn) {
  return turn.value() * WheelCir;
};

//Drive motor to height
void SubClimber::DriveToDistance(units::meter_t distance) {
    TargetDistance = distance;
    _lClimbMotor.SetMotionProfileTarget(DistanceToTurn(distance));
    _rClimbMotor.SetMotionProfileTarget(DistanceToTurn(distance));
}

//Run motor with power
void SubClimber::Start(double power) {
  _lClimbMotor.Set(power);
  _rClimbMotor.Set(power);
}

//Stop motor
void SubClimber::Stop() {
  _lClimbMotor.StopMotor();
  _rClimbMotor.StopMotor();
}

//Reset motor position to 0
void SubClimber::ZeroClimber() {
    _lClimbMotor.SetPosition(0_tr);
    _rClimbMotor.SetPosition(0_tr);
}

//Get left motor current
double SubClimber::GetLeftCurrent() {
    return _lClimbMotor.GetOutputCurrent();
}

//Get right motor current
double SubClimber::GetRightCurrent() {
    return _rClimbMotor.GetOutputCurrent();
}

units::meter_t SubClimber::GetLeftHeight() {
    return TurnToDistance(_lClimbMotor.GetPosition());
}

units::meter_t SubClimber::GetRightHeight() {
    return TurnToDistance(_rClimbMotor.GetPosition());
}

//Enable or disable top and bottom limit
void SubClimber::EnableSoftLimit(bool enabled) {
  rev::spark::SparkBaseConfig config;
  config.softLimit.ForwardSoftLimitEnabled(enabled).ReverseSoftLimitEnabled(enabled);
  _lClimbMotor.AdjustConfig(config);
  _rClimbMotor.AdjustConfig(config);
}

//Joystick drive both motor
frc2::CommandPtr SubClimber::ClimberJoystickDrive(frc2::CommandXboxController& _controller) {
    return Run([this, &_controller] {
        _lClimbMotor.Set(-_controller.GetLeftY());
        _rClimbMotor.Set(-_controller.GetLeftY());
    }).FinallyDo([this] {
        _lClimbMotor.SetPositionTarget(_lClimbMotor.GetPosition());
        _rClimbMotor.SetPositionTarget(_rClimbMotor.GetPosition());
    });
}

//Joystick drive left motor only
frc2::CommandPtr SubClimber::ClimberJoystickDriveLeft(frc2::CommandXboxController& _controller) {
    return Run([this, &_controller] {
        _lClimbMotor.Set(-_controller.GetLeftY());
    }).FinallyDo([this] {
        _lClimbMotor.SetPositionTarget(_lClimbMotor.GetPosition());
    });
}

//Joystick drive right motor only
frc2::CommandPtr SubClimber::ClimberJoystickDriveRight(frc2::CommandXboxController& _controller) {
    return Run([this, &_controller] {
        _rClimbMotor.Set(-_controller.GetRightY());
    }).FinallyDo([this] {
        _rClimbMotor.SetPositionTarget(_rClimbMotor.GetPosition());
    });
}

//Ptr cmd of DriveToDistance()
frc2::CommandPtr SubClimber::ClimberPosition(units::meter_t distance) {
    return frc2::cmd::RunOnce([this,distance] {SubClimber::GetInstance().DriveToDistance(distance);});
}

//Ptr cmd of Start()
frc2::CommandPtr SubClimber::ClimberManualDrive(double power) {
    power = std::clamp(power, -1.0, 1.0);
    return frc2::cmd::RunOnce([power] {SubClimber::GetInstance().Start(power);});
}

//Ptr cmd of Stop()
frc2::CommandPtr SubClimber::ClimberStop() {
    return frc2::cmd::RunOnce([this] {SubClimber::GetInstance().Stop();});
}

//Ptr cmd of ZeroClimber()
frc2::CommandPtr SubClimber::ClimberResetZero() {
    return frc2::cmd::RunOnce([] {SubClimber::GetInstance().ZeroClimber();});
}

frc2::CommandPtr SubClimber::ClimberResetTop() {
    return frc2::cmd::RunOnce([this] {_lClimbMotor.SetPosition(30_tr);
    _rClimbMotor.SetPosition(30_tr);});
}

//Auto climber reset by bringing both hook to zero position then reset
frc2::CommandPtr SubClimber::ClimberAutoReset() {
    return frc2::cmd::RunOnce([this] { Reseting = true; EnableSoftLimit(false);})
        .AndThen(ClimberManualDrive(-0.2))
        .AndThen(frc2::cmd::Wait(0.5_s))
        .AndThen(ClimberResetCheck())
        .AndThen(ClimberResetZero())
        .AndThen(ClimberStop())
        .FinallyDo([this] {Reseting = false; Reseted = true; EnableSoftLimit(false); Stop();});
}

//Check if hook touch the bottom
frc2::CommandPtr SubClimber::ClimberResetCheck() {
    return frc2::cmd::RunOnce ([this] {ResetLeft = false; ResetRight = false;})
    .AndThen(
    frc2::cmd::Run([this] {
        
        if (GetLeftCurrent() > currentLimit && !ResetLeft) {
            _lClimbMotor.StopMotor(); ResetLeft = true;
        }
        if (GetRightCurrent() > currentLimit && !ResetRight) {
            _rClimbMotor.StopMotor(); ResetRight = true;
        }
        if (ResetLeft && ResetRight) {
            Reseting = false;
        }
    }).Until([this] { return ResetLeft && ResetRight; }));
}

units::meter_t SubClimber::CheckLeftClimberPos() {
  return TurnToDistance(_lClimbMotor.GetPosition());
}

units::meter_t SubClimber::CheckRightClimberPos() {
  return TurnToDistance(_rClimbMotor.GetPosition());
}